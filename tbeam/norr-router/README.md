# Norr Router - T-Beam 1.1

Transparent LoRa packet repeater for extending mesh network coverage.

## Purpose

Leave these in the terrain to extend/relay the Norr mesh network.
No configuration needed - just power on.

## Hardware

| Component | Model |
|-----------|-------|
| Board | LILYGO T-Beam v1.1 |
| MCU | ESP32-D0WDQ6-V3 (240MHz, 4MB flash) |
| LoRa | Semtech SX1276 (868 MHz) |
| PMU | AXP2101 |
| Display | SSD1306 128x64 OLED |
| Power | USB-C or 18650 battery |

### Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| LoRa SCK | 5 | SPI clock |
| LoRa MISO | 19 | SPI data in |
| LoRa MOSI | 27 | SPI data out |
| LoRa CS | 18 | SPI chip select |
| LoRa RST | 23 | Radio reset |
| LoRa DIO0 | 26 | RX/TX done interrupt |
| I2C SDA | 21 | AXP2101 + OLED |
| I2C SCL | 22 | AXP2101 + OLED |
| Button | 38 | Display on/off toggle |
| LED | 14 | Disabled (only charging LED active) |

### Power Rails (AXP2101)

| Rail | Voltage | Function |
|------|---------|----------|
| ALDO2 | 3.3V | LoRa SX1276 |
| CHG LED | - | Hardware controlled (charging indicator) |

## Features

- **Transparent relay** - No identity, no routing table
- **Dedup buffer** - 32 entries, 10s TTL (avoids echo loops)
- **Hop count** - Increments on relay, drops at MAX_HOPS (16)
- **Collision avoidance** - Random 50-150ms delay before TX
- **OLED display** - Norr logo, "Router", version, RX/TX, peers
- **Button toggle** - Press GPIO 38 button to toggle display on/off
- **No LEDs** - Only AXP charging LED active

## Packet Flow

```
LoRa RX → Format check → Dedup check → Hop check → Hop++ → Random delay → LoRa TX
              ↓              ↓            ↓
            DROP           DROP         DROP
```

## LoRa Configuration

| Parameter | Value |
|-----------|-------|
| Frequency | 865 MHz |
| Bandwidth | 125 kHz |
| Spreading Factor | 9 |
| Coding Rate | 4/5 |
| Sync Word | 0x12 |
| TX Power | 17 dBm |

## Display Layout

```
┌─────────────────────┐
│[LOGO] Router        │
│       v1.1 865MHz   │
│       RX:42 TX:38   │
│       Peers:3 Drop:4│
└─────────────────────┘
```

## Build & Flash

```bash
cd norr-firmware/tbeam/norr-router

# Build
pio run

# Flash (adjust port as needed)
pio run -t upload --upload-port /dev/ttyACM0

# Monitor serial output
pio device monitor
```

## Serial Output

```
=================================
 Norr Router v1.1
 T-Beam 1.1 - Transparent Relay
=================================
[PWR] AXP2101 OK
[LoRa] Initializing SX1276... OK
[LoRa] 865.00 MHz, SF9, BW 125.00
[Router] Listening...
[OLED] OK
[RELAY] 121 bytes, hop 0→1, delay 87ms
[RELAY] 121 bytes, hop 1→2, delay 124ms
[DROP] Duplicate
[DROP] Hop limit (16 hops)
```
