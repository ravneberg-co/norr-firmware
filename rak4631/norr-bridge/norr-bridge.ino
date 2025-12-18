/**
 * Norr LoRa Bridge Firmware for RAK4631
 *
 * Simple serial <-> LoRa bridge for Norr mesh protocol
 * Uses SX126x-Arduino library (official RAK library)
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

// Buffers
static uint8_t loraBuffer[MAX_PAYLOAD];
static RadioEvents_t RadioEvents;
static volatile bool txDone = true;

// Forward declarations
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);
void handleSerialInput();
uint16_t calculateCRC(uint8_t* data, size_t len);
void sendToSerial(uint8_t* data, size_t len);

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

  Serial.println("\n[Norr LoRa Bridge v2.2]");
  Serial.println("Using SX126x-Arduino library");
  Serial.println("Initializing...");

  // Initialize LoRa chip (this handles all RAK4631 specific pins internally)
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
}

void loop() {
  // Handle any pending Radio events
  Radio.IrqProcess();

  // Handle incoming serial data
  handleSerialInput();
}

// LoRa TX Done callback
void OnTxDone(void) {
  Serial.println("[TX] Done");
  txDone = true;

  // CRITICAL: Wait for radio to fully complete TX->RX mode transition
  // SX126x needs 5-15ms to switch modes. Without this delay, the radio
  // will miss packets sent immediately after our transmission.
  delay(20);  // 20ms ensures stable RX readiness

  // Return to receive mode
  Radio.Rx(RX_TIMEOUT_VALUE);
}

// LoRa RX Done callback
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  if (size > 0 && size <= MAX_PAYLOAD) {
    // Send to serial
    sendToSerial(payload, size);

    Serial.print("[RX] ");
    Serial.print(size);
    Serial.print(" bytes, RSSI: ");
    Serial.print(rssi);
    Serial.print(", SNR: ");
    Serial.println(snr);
  }

  // Continue receiving
  Radio.Rx(RX_TIMEOUT_VALUE);
}

// LoRa TX Timeout callback
void OnTxTimeout(void) {
  Serial.println("[TX] Timeout");
  txDone = true;

  // Wait for radio mode transition (same as OnTxDone)
  delay(20);

  Radio.Rx(RX_TIMEOUT_VALUE);
}

// LoRa RX Timeout callback
void OnRxTimeout(void) {
  // This shouldn't happen with RX_TIMEOUT_VALUE = 0
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

          // Protect txDone access with critical section to prevent race condition
          // between ISR (OnTxDone) and main loop
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
          } else if (!canSend) {
            Serial.println("[TX] Busy");
          } else {
            Serial.println("[CRC ERR]");
          }
        }
        state = WAIT_START;
        break;
    }
  }

  // Timeout - reset state machine
  if (state != WAIT_START && millis() - lastByte > SERIAL_TIMEOUT) {
    state = WAIT_START;
  }
}
