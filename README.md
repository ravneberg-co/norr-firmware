# Norr Firmware

Firmware for hardware devices used with the Norr mesh protocol.

## Structure

```
norr-firmware/
├── rak4631/                     # RAK4631 (nRF52840 + SX1262)
│   └── norr-bridge/             # Serial-to-LoRa bridge
├── esp32/                       # ESP32 (future)
└── rp2040/                      # RP2040 (future)
```

## Supported Devices

| Device | MCU | Radio | Firmware |
|--------|-----|-------|----------|
| RAK4631 | nRF52840 | SX1262 | [norr-bridge](rak4631/norr-bridge/) |

## Quick Start

### RAK4631

1. Build firmware:
   ```bash
   cd rak4631/norr-bridge
   arduino-cli compile --fqbn rakwireless:nrf52:WisCoreRAK4631Board
   ```

2. Upload (double-tap RESET, then):
   ```bash
   cp build/*.uf2 /Volumes/RAK4631/
   ```

## Protocol

All firmwares use the same serial frame format:

```
[0xAA][LEN_HI][LEN_LO][PAYLOAD...][CRC_HI][CRC_LO][0x55]
```

- `0xAA` - Start marker
- `LEN` - 2 bytes, big endian payload length
- `PAYLOAD` - Up to 200 bytes
- `CRC` - CRC16-CCITT of payload
- `0x55` - End marker
