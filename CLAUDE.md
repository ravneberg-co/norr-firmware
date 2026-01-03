# Norr LoRa Bridge Firmware - Development Guide

## Project Overview

This is a LoRa bridge firmware for RAK4631 modules. It acts as a transparent serial-to-LoRa bridge for the Norr mesh protocol.

## Architecture

```
Serial (USB) ◄──► Frame Parser ◄──► SX1262 Radio ◄──► LoRa RF
              │                  │
              │   State Machine  │   Event Callbacks
              │   (handleSerial) │   (OnTxDone, OnRxDone)
```

The firmware is **payload-agnostic** - it does not interpret Norr protocol messages. All encryption, signatures, and mesh routing happen in the Norr library on the host.

## Key Files

- `norr-firmware.ino` - Main firmware source
- `test_lora.py` - Python test script for hardware verification
- `build/` - Compiled binaries (UF2, HEX, ELF)

## Protocol

Frame format: `[0xAA][LEN_HI][LEN_LO][PAYLOAD][CRC_HI][CRC_LO][0x55]`

The same format is used in `norr/src/transport/lora.rs`.

## Radio Configuration

Currently hardcoded for NZ_865 (865 MHz). To change region, modify these defines:

```cpp
#define RF_FREQUENCY 865000000  // Hz
#define TX_OUTPUT_POWER 22      // dBm (max for SX1262)
```

## Build

Uses Arduino toolchain with RAKwireless nRF52 BSP. Dependencies:
- RAKwireless nRF52 BSP v1.3.3+
- SX126x-Arduino library v2.0.32+

## Important Notes

- The `txDone` flag is protected with `noInterrupts()`/`interrupts()` to prevent race conditions between ISR and main loop
- Continuous RX mode (RX_TIMEOUT_VALUE = 0) keeps radio always listening
- Debug output goes to same serial port as data - the Norr library handles frame sync

## Related Code

- Norr transport layer: `../norr/src/transport/lora.rs`
- Norr LoRa example: `../norr/examples/lora_demo.rs`
