# Norr LoRa Bridge Firmware

Serial-to-LoRa bridge firmware for RAK4631 (nRF52840 + SX1262) modules, designed for the Norr mesh protocol.

## Hardware

- **Module**: RAKwireless RAK4631 WisBlock Core
- **MCU**: Nordic nRF52840 (ARM Cortex-M4F @ 64 MHz)
- **Radio**: Semtech SX1262 LoRa transceiver
- **Antenna**: 865 MHz antenna required (e.g., RAK IPEX antenna)

## Radio Configuration

| Parameter | Value |
|-----------|-------|
| Frequency | 865 MHz (NZ_865 ISM band) |
| TX Power | +22 dBm (max for SX1262) |
| Bandwidth | 125 kHz |
| Spreading Factor | SF9 |
| Coding Rate | 4/5 |
| Max Payload | 200 bytes |

Estimated range: 2-5 km urban, 15+ km rural (line of sight).

## Installation

### Prerequisites

1. Arduino IDE 1.8+ or Arduino CLI
2. RAKwireless nRF52 BSP:
   - Add to Arduino Board Manager URL: `https://raw.githubusercontent.com/RAKWireless/RAKwireless-Arduino-BSP-Index/main/package_rakwireless_index.json`
   - Install "RAKwireless nRF52 Boards"
3. SX126x-Arduino library (via Library Manager)

### Flashing

**Option 1: Arduino IDE**
1. Select board: `RAK4631`
2. Select port: `/dev/cu.usbmodemXXXX` (macOS) or `COMx` (Windows)
3. Upload sketch

**Option 2: UF2 Bootloader**
1. Double-tap reset button on RAK4631 (enters bootloader mode)
2. RAK4631 appears as USB drive
3. Drag `build/norr-lora-bridge.ino.uf2` to the drive

## Serial Protocol

The firmware implements a simple framing protocol for reliable serial communication:

```
[START][LEN_HI][LEN_LO][PAYLOAD...][CRC_HI][CRC_LO][END]
  0xAA   2 bytes       0-200 bytes    2 bytes      0x55
```

- **Start marker**: `0xAA`
- **Length**: 16-bit big-endian payload length
- **Payload**: Raw bytes (max 200)
- **CRC**: CRC16-CCITT (poly 0x1021, init 0xFFFF)
- **End marker**: `0x55`

### Serial Settings

- Baud rate: 115200
- Data bits: 8
- Parity: None
- Stop bits: 1

## Integration with Norr

The firmware acts as a transparent bridge - it does not interpret Norr protocol messages. All encryption, signatures, and mesh routing are handled by the Norr library on the host.

```
┌─────────────────┐     Serial      ┌─────────────────┐     LoRa RF     ┌─────────────────┐
│  Norr Library   │◄──────────────►│  RAK4631 Bridge │◄──────────────►│  Other Nodes    │
│  (Host PC/SBC)  │    115200 baud │   (This FW)     │    865 MHz     │                 │
└─────────────────┘                 └─────────────────┘                 └─────────────────┘
```

### Usage with Norr

```bash
# Run the LoRa demo (from norr repository)
cargo run --example lora_demo -- /dev/cu.usbmodem1201
```

## Testing

A Python test script is included for verifying communication between two devices:

```bash
# Connect two RAK4631 devices
python test_lora.py
```

The script sends test messages and verifies bidirectional communication.

## Serial Monitor Output

When running, the firmware outputs status messages:

```
[Norr LoRa Bridge v2.1]
Using SX126x-Arduino library
Initializing...
LoRa init: OK
Radio init: OK
Start RX: OK
================
Ready. Listening...
Freq: 865.00 MHz, SF9, BW 125 kHz
================
[TX] Sending 64 bytes...
[TX] Done
[RX] 64 bytes, RSSI: -45, SNR: 10
```

## Memory Usage

| Section | Usage | Available |
|---------|-------|-----------|
| Flash | ~70 KB (6.7%) | 1 MB |
| RAM | ~10 KB (3.6%) | 256 KB |

## Troubleshooting

**No serial output after flashing**
- Check USB cable supports data (not charge-only)
- Try different USB port
- Double-tap reset to enter bootloader mode

**[TX] Busy messages**
- Previous transmission still in progress
- Reduce send rate or increase TX_TIMEOUT_VALUE

**Poor range**
- Ensure antenna is connected
- Check antenna is rated for 865 MHz
- Avoid metal enclosures

## License

Part of the Norr project.
