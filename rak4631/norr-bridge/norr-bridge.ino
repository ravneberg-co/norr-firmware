/**
 * Norr LoRa Bridge Firmware for RAK4631
 *
 * Serial <-> LoRa bridge with OLED display (RAK1921)
 *
 * Protocol:
 *   Serial RX -> LoRa TX (broadcast)
 *   LoRa RX -> Serial TX
 *
 * Frame format (both directions):
 *   [0xAA][LENGTH_HI][LENGTH_LO][PAYLOAD...][CRC_HI][CRC_LO][0x55]
 */

#include <Arduino.h>
#include <SX126x-Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Firmware version
#define FW_VERSION "2.5"  // Added reactive flow control (READY signal)

// OLED Configuration (RAK1921 - SSD1306 128x64)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// LoRa Configuration
#define RF_FREQUENCY 865000000  // Hz (NZ_865 - New Zealand 865 MHz ISM band)
#define TX_OUTPUT_POWER 22      // dBm (max for SX1262)
#define LORA_BANDWIDTH 0        // 0 = 125 kHz
#define LORA_SPREADING_FACTOR 9 // SF7-SF12
#define LORA_CODINGRATE 1       // 1 = 4/5
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define TX_TIMEOUT_VALUE 3000
#define RX_TIMEOUT_VALUE 0      // Continuous receive

// Protocol constants
#define START_MARKER    0xAA
#define END_MARKER      0x55
#define MAX_PAYLOAD     200
#define SERIAL_TIMEOUT  100     // ms

// Display update interval
#define DISPLAY_UPDATE_MS 250

// Buffers
static uint8_t loraBuffer[MAX_PAYLOAD];
static RadioEvents_t RadioEvents;
static volatile bool txDone = true;

// OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static bool oledAvailable = false;

// Statistics
static uint32_t txCount = 0;
static uint32_t rxCount = 0;
static uint32_t queueCount = 0;
static int16_t lastRssi = 0;
static int8_t lastSnr = 0;

// Activity indicators (flash for 200ms)
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

// Norr logo bitmap (15x32 pixels) - actual Norr logo
#define LOGO_WIDTH 15
#define LOGO_HEIGHT 32
static const unsigned char PROGMEM norr_logo_vertical[] = {
  0x00, 0x00,  // row 0
  0x00, 0x00,  // row 1
  0x01, 0x00,  // row 2
  0x01, 0x00,  // row 3
  0x03, 0x80,  // row 4
  0x03, 0x80,  // row 5
  0x01, 0x00,  // row 6
  0x01, 0x00,  // row 7
  0x01, 0x00,  // row 8
  0xF1, 0x3E,  // row 9
  0x79, 0x1C,  // row 10
  0x7D, 0x1C,  // row 11
  0x7F, 0x1C,  // row 12
  0x7F, 0x1C,  // row 13
  0x7F, 0x1C,  // row 14
  0x77, 0x9C,  // row 15
  0x73, 0xDC,  // row 16
  0x71, 0xFC,  // row 17
  0x71, 0xFC,  // row 18
  0x71, 0xFC,  // row 19
  0x71, 0x7C,  // row 20
  0x71, 0x3C,  // row 21
  0x71, 0x1C,  // row 22
  0xF9, 0x1E,  // row 23
  0x01, 0x00,  // row 24
  0x01, 0x00,  // row 25
  0x03, 0x80,  // row 26
  0x03, 0x80,  // row 27
  0x03, 0x80,  // row 28
  0x02, 0x80,  // row 29
  0x00, 0x00,  // row 30
  0x00, 0x00   // row 31
};

// Forward declarations
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);
void handleSerialInput();
uint16_t calculateCRC(uint8_t* data, size_t len);
void sendToSerial(uint8_t* data, size_t len);
void initOLED();
void updateDisplay();
void drawStartupScreen();

void setup() {
  // Initialize serial
  Serial.begin(115200);

  time_t timeout = millis();
  while (!Serial) {
    if ((millis() - timeout) < 3000) {
      delay(100);
    } else {
      break;
    }
  }

  Serial.println("\n[Norr LoRa Bridge v" FW_VERSION "]");

  // Initialize OLED
  initOLED();

  // Initialize LoRa
  Serial.print("LoRa init: ");
  lora_rak4630_init();
  Serial.println("OK");

  // Initialize Radio callbacks
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  RadioEvents.CadDone = NULL;

  // Initialize the Radio
  Serial.print("Radio init: ");
  Radio.Init(&RadioEvents);
  Serial.println("OK");

  // Set Radio channel
  Radio.SetChannel(RF_FREQUENCY);

  // Set Radio TX configuration
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

  // Set Radio RX configuration
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  // Start continuous receive
  Serial.print("Start RX: ");
  Radio.Rx(RX_TIMEOUT_VALUE);
  Serial.println("OK");

  Serial.println("================");
  Serial.println("Ready. Listening...");
  Serial.print("Freq: ");
  Serial.print(RF_FREQUENCY / 1000000.0);
  Serial.print(" MHz, SF");
  Serial.print(LORA_SPREADING_FACTOR);
  Serial.println(", BW 125 kHz");
  Serial.println("================");

  // Show startup complete on OLED
  if (oledAvailable) {
    delay(1000);  // Show startup screen a bit longer
    updateDisplay();
  }
}

void loop() {
  // Handle any pending Radio events
  Radio.IrqProcess();

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
  Wire.begin();

  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    oledAvailable = true;
    Serial.println("OLED: OK");
    drawStartupScreen();
  } else {
    oledAvailable = false;
    Serial.println("OLED: Not found");
  }
}

// Draw startup screen with logo
void drawStartupScreen() {
  display.clearDisplay();

  // Draw vertical Norr logo on left side (centered vertically)
  display.drawBitmap(0, 16, norr_logo_vertical, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);

  // Vertical line separator
  display.drawLine(18, 0, 18, 63, SSD1306_WHITE);

  // "NORR" text
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(28, 8);
  display.print("NORR");

  // Version and status
  display.setTextSize(1);
  display.setCursor(24, 32);
  display.print("LoRa Bridge v");
  display.print(FW_VERSION);

  display.setCursor(32, 48);
  display.print("Initializing...");

  display.display();
}

// Update display with current status
void updateDisplay() {
  display.clearDisplay();

  // === Left sidebar: Norr logo (centered) ===
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

  // Content area starts at x=30, right edge with 2px margin
  const int X = 30;
  const int R = 125;  // Right edge with margin

  // === Row 1: Frequency and SF ===
  display.setCursor(X, 0);
  display.print("865MHz");
  display.setCursor(R - 17, 0);  // "SF9" aligned right
  display.print("SF");
  display.print(LORA_SPREADING_FACTOR);

  // === Row 2: TX/RX ===
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

  display.setCursor(R - 36, 12);  // "↓RX:0" = 6 chars * 6px = 36
  if (now < rxFlashUntil) {
    display.fillRect(R - 38, 11, 40, 9, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  }
  display.print((char)25);
  display.print("RX:");
  display.print(rxCount);
  display.setTextColor(SSD1306_WHITE);

  // === Row 3: RSSI/SNR ===
  display.setCursor(X, 24);
  display.print("RSSI:");
  display.print(lastRssi);
  display.setCursor(R - 36, 24);  // "SNR:0" = 6 chars * 6px = 36
  display.print("SNR:");
  display.print(lastSnr);

  // === Divider ===
  display.drawLine(X - 2, 34, 127, 34, SSD1306_WHITE);

  // === Row 4: Status and peers ===
  display.setCursor(X, 40);
  if (!txDone) {
    display.print("SENDING");
  } else {
    display.print("LISTEN");
  }

  display.setCursor(R - 48, 40);  // "Peers:X" = 8 chars * 6px = 48
  display.print("Peers:");
  display.print(peerCount);

  // === Row 5: Uptime and queue ===
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

  display.setCursor(R - 24, 52);  // "Q:0" = 4 chars * 6px = 24
  display.print("Q:");
  display.print(queueCount);

  display.display();
}

// LoRa TX Done callback
void OnTxDone(void) {
  txDone = true;
  txCount++;
  txFlashUntil = millis() + 200;

  delay(20);
  Radio.Rx(RX_TIMEOUT_VALUE);

  // Signal host that we're ready for next packet (reactive flow control)
  Serial.println("READY");
}

// LoRa RX Done callback
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  if (size > 0 && size <= MAX_PAYLOAD) {
    sendToSerial(payload, size);

    rxCount++;
    lastRssi = rssi;
    lastSnr = snr;
    rxFlashUntil = millis() + 200;

    // Track unique peers
    updatePeer(payload, size);

    Serial.print("[RX] ");
    Serial.print(size);
    Serial.print(" bytes, RSSI: ");
    Serial.print(rssi);
    Serial.print(", SNR: ");
    Serial.println(snr);
  }

  Radio.Rx(RX_TIMEOUT_VALUE);
}

// LoRa TX Timeout callback
void OnTxTimeout(void) {
  Serial.println("[TX] Timeout");
  txDone = true;
  delay(20);
  Radio.Rx(RX_TIMEOUT_VALUE);
}

// LoRa RX Timeout callback
void OnRxTimeout(void) {
  Radio.Rx(RX_TIMEOUT_VALUE);
}

// LoRa RX Error callback
void OnRxError(void) {
  Serial.println("[RX] Error");
  Radio.Rx(RX_TIMEOUT_VALUE);
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
          queueCount++;  // Track pending TX
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

          noInterrupts();
          bool canSend = txDone;
          if (canSend) {
            txDone = false;
          }
          interrupts();

          if (calculatedCrc == receivedCrc && canSend) {
            Radio.Send(loraBuffer, payloadLen);
            Serial.print("[TX] Sending ");
            Serial.print(payloadLen);
            Serial.println(" bytes...");
            queueCount--;
          } else if (!canSend) {
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
