/**
 * Norr LoRa Bridge Firmware for LilyGO T-Beam
 *
 * Serial <-> LoRa bridge using RadioLib (supports SX1276/SX1262)
 *
 * Protocol:
 *   Serial RX -> LoRa TX (broadcast)
 *   LoRa RX -> Serial TX
 *
 * Frame format (both directions):
 *   [0xAA][LENGTH_HI][LENGTH_LO][PAYLOAD...][CRC_HI][CRC_LO][0x55]
 *
 * T-Beam v1.1/v1.2 pinout:
 *   LoRa: NSS=18, DIO0=26, RST=23, DIO1=33
 *   OLED: SDA=21, SCL=22 (if installed)
 *   GPS: TX=34, RX=12
 */

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Firmware version
#define FW_VERSION "2.4"

// T-Beam v1.1/v1.2 LoRa pins
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_NSS   18
#define LORA_DIO0  26
#define LORA_RST   14   // T-Beam v1.1
#define LORA_DIO1  33

// OLED Configuration (if connected)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// LoRa Configuration (match RAK4631 settings)
#define RF_FREQUENCY      865.0   // MHz
#define TX_POWER          14      // dBm (SX1276 max is 17)
#define BANDWIDTH         125.0   // kHz
#define SPREADING_FACTOR  9
#define CODING_RATE       5       // 4/5
#define PREAMBLE_LENGTH   8
#define SYNC_WORD         0x12

// Protocol constants
#define START_MARKER    0xAA
#define END_MARKER      0x55
#define MAX_PAYLOAD     200
#define SERIAL_TIMEOUT  100     // ms

// Display update interval
#define DISPLAY_UPDATE_MS 250

// Radio instance (created in setup after SPI.begin)
SX1276 *radio = nullptr;

// OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static bool oledAvailable = false;

// Buffers
static uint8_t loraBuffer[MAX_PAYLOAD];
static uint8_t rxBuffer[MAX_PAYLOAD];
static volatile bool txDone = true;
static volatile bool rxFlag = false;
static volatile int rxLength = 0;

// Statistics
static uint32_t txCount = 0;
static uint32_t rxCount = 0;
static uint32_t queueCount = 0;
static float lastRssi = 0;
static float lastSnr = 0;

// Activity indicators
static unsigned long txFlashUntil = 0;
static unsigned long rxFlashUntil = 0;
static unsigned long lastDisplayUpdate = 0;

// Peer tracking - based on first 16 bytes (DestinationID in Norr wire format)
#define MAX_PEERS 8
#define PEER_SIGNATURE_LEN 16
#define PEER_TIMEOUT_MS 300000  // 5 minutes
struct PeerInfo {
  uint8_t signature[PEER_SIGNATURE_LEN];
  unsigned long lastSeen;
  bool active;
};
static PeerInfo peers[MAX_PEERS];
static uint8_t peerCount = 0;

// Check if peer exists, add if new, update lastSeen
void updatePeer(uint8_t* payload, uint16_t size) {
  if (size < PEER_SIGNATURE_LEN) return;

  unsigned long now = millis();

  // First, expire old peers
  peerCount = 0;
  for (int i = 0; i < MAX_PEERS; i++) {
    if (peers[i].active) {
      if (now - peers[i].lastSeen > PEER_TIMEOUT_MS) {
        peers[i].active = false;
      } else {
        peerCount++;
      }
    }
  }

  // Check if this peer already exists
  for (int i = 0; i < MAX_PEERS; i++) {
    if (peers[i].active && memcmp(peers[i].signature, payload, PEER_SIGNATURE_LEN) == 0) {
      peers[i].lastSeen = now;
      return;  // Already known
    }
  }

  // New peer - find empty slot
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].active) {
      memcpy(peers[i].signature, payload, PEER_SIGNATURE_LEN);
      peers[i].lastSeen = now;
      peers[i].active = true;
      peerCount++;
      return;
    }
  }

  // No empty slot - replace oldest
  int oldest = 0;
  unsigned long oldestTime = peers[0].lastSeen;
  for (int i = 1; i < MAX_PEERS; i++) {
    if (peers[i].lastSeen < oldestTime) {
      oldest = i;
      oldestTime = peers[i].lastSeen;
    }
  }
  memcpy(peers[oldest].signature, payload, PEER_SIGNATURE_LEN);
  peers[oldest].lastSeen = now;
  peers[oldest].active = true;
}

// Norr logo bitmap (15x32 pixels)
#define LOGO_WIDTH 15
#define LOGO_HEIGHT 32
static const unsigned char PROGMEM norr_logo_vertical[] = {
  0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00,
  0x03, 0x80, 0x03, 0x80, 0x01, 0x00, 0x01, 0x00,
  0x01, 0x00, 0xF1, 0x3E, 0x79, 0x1C, 0x7D, 0x1C,
  0x7F, 0x1C, 0x7F, 0x1C, 0x7F, 0x1C, 0x77, 0x9C,
  0x73, 0xDC, 0x71, 0xFC, 0x71, 0xFC, 0x71, 0xFC,
  0x71, 0x7C, 0x71, 0x3C, 0x71, 0x1C, 0xF9, 0x1E,
  0x01, 0x00, 0x01, 0x00, 0x03, 0x80, 0x03, 0x80,
  0x03, 0x80, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00
};

// ISR flags
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setRxFlag(void) {
  rxFlag = true;
}

// Forward declarations
void initOLED();
void updateDisplay();
void handleSerialInput();
uint16_t calculateCRC(uint8_t* data, size_t len);
void sendToSerial(uint8_t* data, size_t len);

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial

  Serial.println("\n[Norr LoRa Bridge T-Beam v" FW_VERSION "]");
  Serial.println("Using RadioLib");

  // Initialize SPI for LoRa (T-Beam uses non-default pins)
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  // Create radio instance AFTER SPI.begin()
  radio = new SX1276(new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1, SPI));

  // Initialize OLED
  initOLED();

  // Initialize LoRa
  Serial.print("LoRa init: ");
  int state = radio->begin(RF_FREQUENCY, BANDWIDTH, SPREADING_FACTOR,
                          CODING_RATE, SYNC_WORD, TX_POWER, PREAMBLE_LENGTH);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("OK");
  } else {
    Serial.print("FAILED: ");
    Serial.println(state);
    // Show error on OLED
    if (oledAvailable) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("LoRa FAILED!");
      display.setCursor(0, 16);
      display.print("Error: ");
      display.print(state);
      display.setCursor(0, 32);
      display.print("Check wiring");
      display.display();
    }
    while (true) { delay(1000); }
  }

  // Set DIO0 action for RX done
  radio->setDio0Action(setRxFlag, RISING);

  // Start receiving
  Serial.print("Start RX: ");
  state = radio->startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("OK");
  } else {
    Serial.print("FAILED: ");
    Serial.println(state);
  }

  Serial.println("================");
  Serial.println("Ready. Listening...");
  Serial.print("Freq: ");
  Serial.print(RF_FREQUENCY);
  Serial.print(" MHz, SF");
  Serial.print(SPREADING_FACTOR);
  Serial.println(", BW 125 kHz");
  Serial.println("================");
}

void loop() {
  // Check for received packet
  if (rxFlag) {
    rxFlag = false;

    int len = radio->getPacketLength();
    if (len > 0 && len <= MAX_PAYLOAD) {
      int state = radio->readData(rxBuffer, len);

      if (state == RADIOLIB_ERR_NONE) {
        // Get signal info
        lastRssi = radio->getRSSI();
        lastSnr = radio->getSNR();

        // Send to serial
        sendToSerial(rxBuffer, len);

        rxCount++;
        rxFlashUntil = millis() + 200;

        // Track unique peers
        updatePeer(rxBuffer, len);

        Serial.print("[RX] ");
        Serial.print(len);
        Serial.print(" bytes, RSSI: ");
        Serial.print(lastRssi);
        Serial.print(", SNR: ");
        Serial.println(lastSnr);
      }
    }

    // Restart receive
    radio->startReceive();
  }

  // Handle incoming serial data
  handleSerialInput();

  // Update display periodically
  if (oledAvailable && (millis() - lastDisplayUpdate > DISPLAY_UPDATE_MS)) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
}

// Initialize OLED display
void initOLED() {
  Wire.begin(21, 22);  // T-Beam I2C pins

  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    oledAvailable = true;
    Serial.println("OLED: OK");

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(20, 20);
    display.print("NORR T-Beam v");
    display.print(FW_VERSION);
    display.setCursor(20, 40);
    display.print("Initializing...");
    display.display();
  } else {
    oledAvailable = false;
    Serial.println("OLED: Not found");
  }
}

// Update display with current status
void updateDisplay() {
  display.clearDisplay();

  // Left sidebar: Norr logo
  display.drawBitmap(0, 8, norr_logo_vertical, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);

  // "NORR" and version below logo
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(1, 44);
  display.print("NORR");
  display.setCursor(1, 54);
  display.print("v");
  display.print(FW_VERSION);

  // Vertical separator line
  display.drawLine(26, 0, 26, 63, SSD1306_WHITE);

  // Content area
  const int X = 30;
  const int R = 125;

  // Row 1: Frequency and SF
  display.setCursor(X, 0);
  display.print("865MHz");
  display.setCursor(R - 17, 0);
  display.print("SF");
  display.print(SPREADING_FACTOR);

  // Row 2: TX/RX
  unsigned long now = millis();

  display.setCursor(X, 12);
  if (now < txFlashUntil) {
    display.fillRect(X - 2, 11, 40, 9, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  }
  display.print((char)24);
  display.print("TX:");
  display.print(txCount);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(R - 36, 12);
  if (now < rxFlashUntil) {
    display.fillRect(R - 38, 11, 40, 9, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  }
  display.print((char)25);
  display.print("RX:");
  display.print(rxCount);
  display.setTextColor(SSD1306_WHITE);

  // Row 3: RSSI/SNR
  display.setCursor(X, 24);
  display.print("RSSI:");
  display.print((int)lastRssi);
  display.setCursor(R - 36, 24);
  display.print("SNR:");
  display.print((int)lastSnr);

  // Divider
  display.drawLine(X - 2, 34, 127, 34, SSD1306_WHITE);

  // Row 4: Status and peers
  display.setCursor(X, 40);
  if (!txDone) {
    display.print("SENDING");
  } else {
    display.print("LISTEN");
  }

  display.setCursor(R - 48, 40);
  display.print("Peers:");
  display.print(peerCount);

  // Row 5: Uptime and queue
  display.setCursor(X, 52);
  unsigned long uptime = millis() / 1000;
  unsigned long hours = uptime / 3600;
  unsigned long mins = (uptime % 3600) / 60;
  unsigned long secs = uptime % 60;
  display.print("Up:");
  if (hours > 0) {
    display.print(hours);
    display.print("h");
  }
  display.print(mins);
  display.print("m");
  display.print(secs);
  display.print("s");

  display.setCursor(R - 24, 52);
  display.print("Q:");
  display.print(queueCount);

  display.display();
}

// Calculate CRC16 (CCITT)
uint16_t calculateCRC(uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// Send data to serial with framing
void sendToSerial(uint8_t* data, size_t len) {
  uint16_t crc = calculateCRC(data, len);

  Serial.write(START_MARKER);
  Serial.write((len >> 8) & 0xFF);
  Serial.write(len & 0xFF);
  Serial.write(data, len);
  Serial.write((crc >> 8) & 0xFF);
  Serial.write(crc & 0xFF);
  Serial.write(END_MARKER);
  Serial.flush();
}

// Handle serial input and send via LoRa
void handleSerialInput() {
  static enum { WAIT_START, READ_LEN_HI, READ_LEN_LO, READ_PAYLOAD, READ_CRC_HI, READ_CRC_LO, WAIT_END } state = WAIT_START;
  static uint16_t payloadLen = 0;
  static uint16_t bytesRead = 0;
  static uint16_t receivedCrc = 0;
  static unsigned long lastByte = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();
    lastByte = millis();

    switch (state) {
      case WAIT_START:
        if (b == START_MARKER) {
          state = READ_LEN_HI;
          bytesRead = 0;
          queueCount++;
        }
        break;

      case READ_LEN_HI:
        payloadLen = (uint16_t)b << 8;
        state = READ_LEN_LO;
        break;

      case READ_LEN_LO:
        payloadLen |= b;
        if (payloadLen > MAX_PAYLOAD) {
          state = WAIT_START;
          queueCount--;
        } else if (payloadLen == 0) {
          state = READ_CRC_HI;
        } else {
          state = READ_PAYLOAD;
        }
        break;

      case READ_PAYLOAD:
        loraBuffer[bytesRead++] = b;
        if (bytesRead >= payloadLen) {
          state = READ_CRC_HI;
        }
        break;

      case READ_CRC_HI:
        receivedCrc = (uint16_t)b << 8;
        state = READ_CRC_LO;
        break;

      case READ_CRC_LO:
        receivedCrc |= b;
        state = WAIT_END;
        break;

      case WAIT_END:
        if (b == END_MARKER) {
          uint16_t calculatedCrc = calculateCRC(loraBuffer, payloadLen);

          if (calculatedCrc == receivedCrc && txDone) {
            txDone = false;
            txFlashUntil = millis() + 200;

            Serial.print("[TX] Sending ");
            Serial.print(payloadLen);
            Serial.println(" bytes...");

            int state = radio->transmit(loraBuffer, payloadLen);

            if (state == RADIOLIB_ERR_NONE) {
              Serial.println("[TX] Done");
              txCount++;
            } else {
              Serial.print("[TX] Failed: ");
              Serial.println(state);
            }

            txDone = true;
            queueCount--;

            // Restart receive
            radio->startReceive();
          } else if (!txDone) {
            Serial.println("[TX] Busy");
          } else {
            Serial.println("[CRC ERR]");
            queueCount--;
          }
        } else {
          queueCount--;
        }
        state = WAIT_START;
        break;
    }
  }

  // Timeout - reset state machine
  if (state != WAIT_START && millis() - lastByte > SERIAL_TIMEOUT) {
    state = WAIT_START;
    if (queueCount > 0) queueCount--;
  }
}
