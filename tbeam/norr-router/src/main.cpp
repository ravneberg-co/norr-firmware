/**
 * Norr Router Firmware for T-Beam 1.1
 *
 * Minimal LoRa packet repeater - completely transparent.
 * No identity, no routing table, just repeat valid packets.
 *
 * Logic:
 *   1. Receive LoRa packet
 *   2. Check format marker (bit 7 of byte 0 must be 1)
 *   3. Check if seen recently (dedup)
 *   4. Check hop count < MAX_HOPS
 *   5. Increment hop count
 *   6. Random delay (avoid collision)
 *   7. Retransmit
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Firmware version
#define FW_VERSION "1.1"

// Set to 1 for verbose debug output
#define DEBUG_VERBOSE 0

// AXP power management (T-Beam)
#define AXP_ADDR 0x34
#define AXP192_CHIP_ID 0x41
#define AXP2101_CHIP_ID 0x4A

// T-Beam 1.1 LoRa pins (from pins_arduino.h)
// LORA_SCK=5, LORA_MISO=19, LORA_MOSI=27, LORA_CS=18, LORA_RST=23, LORA_IO0=26

// LoRa Configuration (must match endpoints)
#define RF_FREQUENCY      865.0   // MHz (NZ_865)
#define BANDWIDTH         125.0   // kHz
#define SPREADING_FACTOR  9
#define CODING_RATE       5       // 4/5
#define SYNC_WORD         0x12
#define TX_POWER          17      // dBm (max for SX1276)

// Router configuration
#define MAX_HOPS          16      // Drop packets with higher hop count
#define DEDUP_SIZE        32      // Number of recent packets to remember
#define DEDUP_TTL_MS      10000   // How long to remember packets (10s)
#define DELAY_MIN_MS      50      // Minimum retransmit delay
#define DELAY_MAX_MS      150     // Maximum retransmit delay
#define MAX_PACKET_SIZE   255

// LED disabled - only AXP charging LED active
// #define LED_PIN           LED_BUILTIN

// Button for display toggle (T-Beam has button on GPIO 38)
#define BUTTON_PIN        38

// OLED display (128x64 SSD1306 on I2C)
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     64
#define OLED_ADDR         0x3C
#define LOGO_WIDTH        16
#define LOGO_HEIGHT       32

// Norr logo bitmap (16x32)
static const unsigned char PROGMEM norr_logo[] = {
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

// Display instance
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool displayOn = true;
volatile bool buttonPressed = false;
unsigned long lastButtonTime = 0;

// Radio instance (SX1276 for T-Beam 1.1 868MHz)
SX1276 *radio = nullptr;

// Receive buffer
volatile bool receivedFlag = false;
uint8_t rxBuffer[MAX_PACKET_SIZE];
int rxLen = 0;

// Dedup buffer
struct DedupEntry {
    uint32_t hash;
    uint32_t timestamp;
};
DedupEntry dedupBuffer[DEDUP_SIZE];
uint8_t dedupIndex = 0;

// Statistics
uint32_t rxCount = 0;
uint32_t txCount = 0;
uint32_t dropDup = 0;
uint32_t dropHop = 0;
uint32_t dropFmt = 0;

// ISR for receive
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void rxISR(void) {
    receivedFlag = true;
}

// Simple CRC32 for dedup hashing
uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

// Check if packet hash was seen recently
bool seenRecently(uint32_t hash) {
    uint32_t now = millis();
    for (int i = 0; i < DEDUP_SIZE; i++) {
        if (dedupBuffer[i].hash == hash &&
            (now - dedupBuffer[i].timestamp) < DEDUP_TTL_MS) {
            return true;
        }
    }
    return false;
}

// Mark packet as seen
void markSeen(uint32_t hash) {
    dedupBuffer[dedupIndex].hash = hash;
    dedupBuffer[dedupIndex].timestamp = millis();
    dedupIndex = (dedupIndex + 1) % DEDUP_SIZE;
}


// Peer tracking (simple: count unique source IDs seen)
#define MAX_PEERS 16
uint8_t peerIds[MAX_PEERS][16];  // Store first 16 bytes (source ID area)
uint8_t peerCount = 0;

void trackPeer(const uint8_t* packet, int len) {
    if (len < 18) return;  // Need at least header + some ID bytes

    // Check if we've seen this source before (bytes 2-17 contain source area)
    for (int i = 0; i < peerCount; i++) {
        if (memcmp(peerIds[i], packet + 2, 16) == 0) {
            return;  // Already known
        }
    }

    // Add new peer
    if (peerCount < MAX_PEERS) {
        memcpy(peerIds[peerCount], packet + 2, 16);
        peerCount++;
    }
}

// Button ISR for display toggle
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void buttonISR() {
    buttonPressed = true;
}

// Update display
void updateDisplay() {
    if (!displayOn) {
        display.clearDisplay();
        display.display();
        return;
    }

    display.clearDisplay();

    // Logo on left side
    display.drawBitmap(0, 16, norr_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);

    // Vertical separator
    display.drawLine(20, 0, 20, 63, SSD1306_WHITE);

    // Content area starts at x=24
    display.setTextColor(SSD1306_WHITE);

    // Row 1: "Router" title
    display.setTextSize(2);
    display.setCursor(24, 0);
    display.print("Router");

    display.setTextSize(1);

    // Row 2: Version
    display.setCursor(24, 20);
    display.print("v");
    display.print(FW_VERSION);
    display.print(" ");
    display.print((int)RF_FREQUENCY);
    display.print("MHz");

    // Row 3: RX/TX
    display.setCursor(24, 34);
    display.print("RX:");
    display.print(rxCount);
    display.print(" TX:");
    display.print(txCount);

    // Row 4: Peers and drops
    display.setCursor(24, 48);
    display.print("Peers:");
    display.print(peerCount);
    display.print(" Drop:");
    display.print(dropDup + dropHop + dropFmt);

    display.display();
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=================================");
    Serial.println(" Norr Router v" FW_VERSION);
    Serial.println(" T-Beam 1.1 - Transparent Relay");
    Serial.println("=================================");

    // Initialize I2C for AXP power management
    Wire.begin(21, 22);  // T-Beam I2C pins

    // Enable LoRa power on AXP (supports both AXP192 and AXP2101)
    Serial.print("[PWR] ");
    Wire.beginTransmission(AXP_ADDR);
    if (Wire.endTransmission() == 0) {
        // Read chip ID
        Wire.beginTransmission(AXP_ADDR);
        Wire.write(0x03);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)AXP_ADDR, (uint8_t)1);
        uint8_t chipId = Wire.read();

        if (chipId == AXP2101_CHIP_ID) {
            // AXP2101: Enable ALDO2 for LoRa (3.3V)
            Wire.beginTransmission(AXP_ADDR);
            Wire.write(0x93);  // ALDO2 voltage register
            Wire.write(0x1C);  // 3.3V
            Wire.endTransmission();

            Wire.beginTransmission(AXP_ADDR);
            Wire.write(0x90);
            Wire.endTransmission(false);
            Wire.requestFrom((uint8_t)AXP_ADDR, (uint8_t)1);
            uint8_t aldoEn = Wire.read();

            Wire.beginTransmission(AXP_ADDR);
            Wire.write(0x90);
            Wire.write(aldoEn | 0x02);  // Enable ALDO2
            Wire.endTransmission();

            // Disable CHGLED blinking - only on when charging
            // Register 0x69: bits[1:0] = 00 off, 01 blink1Hz, 10 blink4Hz, 11 on
            Wire.beginTransmission(AXP_ADDR);
            Wire.write(0x69);
            Wire.write(0x00);  // LED off (charging LED still works via hardware)
            Wire.endTransmission();

            Serial.println("AXP2101 OK");
        } else {
            // AXP192: Enable LDO2 for LoRa
            Wire.beginTransmission(AXP_ADDR);
            Wire.write(0x28);
            Wire.write(0xCC);  // LDO2=3.3V
            Wire.endTransmission();

            Wire.beginTransmission(AXP_ADDR);
            Wire.write(0x12);
            Wire.endTransmission(false);
            Wire.requestFrom((uint8_t)AXP_ADDR, (uint8_t)1);
            uint8_t pwr = Wire.read();

            Wire.beginTransmission(AXP_ADDR);
            Wire.write(0x12);
            Wire.write(pwr | 0x04);  // Enable LDO2
            Wire.endTransmission();

            Serial.println("AXP192 OK");
        }
    } else {
        Serial.println("NO AXP - power may fail!");
    }
    delay(500);  // Let power stabilize

    // Ensure all LEDs are off (GPIO 4 and 14 are used on different T-Beam versions)
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);
    pinMode(14, OUTPUT);
    digitalWrite(14, LOW);

    // Reset LoRa module
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(100);

    // Set up CS pin and SPI
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);

    SPIClass *loraSpi = new SPIClass(VSPI);
    loraSpi->begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

    // Create radio module (SX1276: CS, DIO0, RST, SPI)
    Module* mod = new Module(LORA_CS, LORA_IO0, LORA_RST, RADIOLIB_NC, *loraSpi);
    radio = new SX1276(mod);

    // Initialize radio
    Serial.print("[LoRa] Initializing SX1276... ");
    int state = radio->begin(
        RF_FREQUENCY,
        BANDWIDTH,
        SPREADING_FACTOR,
        CODING_RATE,
        SYNC_WORD,
        TX_POWER
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("FAILED! Code: ");
        Serial.println(state);
        Serial.println("[LoRa] Check wiring and power");
        while (true) {
            delay(1000);  // Halt on error
        }
    }
    Serial.println("OK");

    // Configure for continuous RX (SX1276 uses DIO0 for RxDone)
    radio->setDio0Action(rxISR, RISING);

    Serial.print("[LoRa] ");
    Serial.print(RF_FREQUENCY);
    Serial.print(" MHz, SF");
    Serial.print(SPREADING_FACTOR);
    Serial.print(", BW ");
    Serial.println(BANDWIDTH);
    Serial.println("[Router] Listening...");

    // Start receiving
    radio->startReceive();

    // Clear dedup buffer
    memset(dedupBuffer, 0, sizeof(dedupBuffer));

    // Initialize OLED display
    Serial.print("[OLED] ");
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OK");
        display.clearDisplay();
        display.display();
        updateDisplay();
    } else {
        Serial.println("Not found");
    }

    // Button for display toggle
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
}

void loop() {
    // Handle button press (debounce 200ms)
    if (buttonPressed) {
        unsigned long now = millis();
        if (now - lastButtonTime > 200) {
            displayOn = !displayOn;
            updateDisplay();
            lastButtonTime = now;
        }
        buttonPressed = false;
    }

    // Periodic display update (every 1s)
    static uint32_t lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 1000) {
        lastDisplayUpdate = millis();
        updateDisplay();
    }

    if (!receivedFlag) {
        return;
    }

    receivedFlag = false;

    // Read packet
    rxLen = radio->getPacketLength();
    if (rxLen <= 0 || rxLen > MAX_PACKET_SIZE) {
        radio->startReceive();
        return;
    }

    int state = radio->readData(rxBuffer, rxLen);
    if (state != RADIOLIB_ERR_NONE) {
        radio->startReceive();
        return;
    }

    rxCount++;
    trackPeer(rxBuffer, rxLen);

    // Need at least 2 bytes for header
    if (rxLen < 2) {
        radio->startReceive();
        return;
    }

    // Check format marker (bit 7 of byte 0 must be 1)
    if (!(rxBuffer[0] & 0x80)) {
        dropFmt++;
        Serial.print("[DROP] Invalid format (");
        Serial.print(rxLen);
        Serial.println(" bytes)");
        radio->startReceive();
        return;
    }

    // Get hop count from byte 1
    uint8_t hopCount = rxBuffer[1];

    // Too many hops?
    if (hopCount >= MAX_HOPS) {
        dropHop++;
        Serial.print("[DROP] Hop limit (");
        Serial.print(hopCount);
        Serial.println(" hops)");
        radio->startReceive();
        return;
    }

    // Calculate hash for dedup
    uint32_t hash = crc32(rxBuffer, rxLen);

    // Seen recently?
    if (seenRecently(hash)) {
        dropDup++;
        Serial.println("[DROP] Duplicate");
        radio->startReceive();
        return;
    }

    // Mark as seen (before modifying packet)
    markSeen(hash);

    // Increment hop count
    rxBuffer[1] = hopCount + 1;

    // Random delay to avoid collision with other routers
    int delayMs = random(DELAY_MIN_MS, DELAY_MAX_MS);
    delay(delayMs);

    // Retransmit
    state = radio->transmit(rxBuffer, rxLen);

    if (state == RADIOLIB_ERR_NONE) {
        txCount++;
        Serial.print("[RELAY] ");
        Serial.print(rxLen);
        Serial.print(" bytes, hop ");
        Serial.print(hopCount);
        Serial.print("→");
        Serial.print(hopCount + 1);
        Serial.print(", delay ");
        Serial.print(delayMs);
        Serial.println("ms");
    } else {
        Serial.print("[ERROR] TX failed: ");
        Serial.println(state);
    }

    // Resume receiving
    radio->startReceive();

    // Print stats periodically
    static uint32_t lastStats = 0;
    if (millis() - lastStats > 30000) {
        lastStats = millis();
        Serial.println();
        Serial.println("--- Stats ---");
        Serial.print("RX: "); Serial.println(rxCount);
        Serial.print("TX: "); Serial.println(txCount);
        Serial.print("Drop (dup): "); Serial.println(dropDup);
        Serial.print("Drop (hop): "); Serial.println(dropHop);
        Serial.print("Drop (fmt): "); Serial.println(dropFmt);
        Serial.println("-------------");
        Serial.println();
    }
}
