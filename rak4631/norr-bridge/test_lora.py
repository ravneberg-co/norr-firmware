#!/usr/bin/env python3
"""
Test script for Norr LoRa Bridge
Sends a message from one RAK4631 and listens on the other
"""

import serial
import time
import struct
import threading
import sys

# Serial ports for the two RAK4631 devices
PORT1 = '/dev/cu.usbmodem1201'
PORT2 = '/dev/cu.usbmodem1301'
BAUD = 115200

# Protocol constants
START_MARKER = 0xAA
END_MARKER = 0x55

def crc16(data):
    """Calculate CRC16 CCITT"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def frame_message(payload):
    """Frame a message with start/end markers and CRC"""
    if isinstance(payload, str):
        payload = payload.encode('utf-8')

    length = len(payload)
    crc = crc16(payload)

    frame = bytes([START_MARKER])
    frame += struct.pack('>H', length)  # Big endian length
    frame += payload
    frame += struct.pack('>H', crc)     # Big endian CRC
    frame += bytes([END_MARKER])

    return frame

def read_serial(ser, name, stop_event, received_messages):
    """Read from serial port and print received data"""
    buffer = b''

    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                buffer += data

                # Try to parse framed messages
                while len(buffer) >= 7:  # Minimum frame size
                    # Look for start marker
                    start_idx = buffer.find(bytes([START_MARKER]))
                    if start_idx == -1:
                        buffer = b''
                        break

                    if start_idx > 0:
                        # Print any debug text before the frame
                        debug_text = buffer[:start_idx].decode('utf-8', errors='ignore').strip()
                        if debug_text:
                            print(f"[{name}] {debug_text}")
                        buffer = buffer[start_idx:]

                    if len(buffer) < 3:
                        break

                    # Get length
                    length = struct.unpack('>H', buffer[1:3])[0]
                    frame_size = 1 + 2 + length + 2 + 1  # start + len + payload + crc + end

                    if len(buffer) < frame_size:
                        break

                    # Verify end marker
                    if buffer[frame_size - 1] == END_MARKER:
                        payload = buffer[3:3+length]
                        received_crc = struct.unpack('>H', buffer[3+length:3+length+2])[0]
                        calculated_crc = crc16(payload)

                        if received_crc == calculated_crc:
                            msg = payload.decode('utf-8', errors='ignore')
                            print(f"[{name}] ✓ Received via LoRa: '{msg}'")
                            received_messages.append(msg)
                        else:
                            print(f"[{name}] ✗ CRC error")

                        buffer = buffer[frame_size:]
                    else:
                        # Invalid frame, skip start marker
                        buffer = buffer[1:]

                # Print any remaining text (debug output)
                if buffer and START_MARKER not in buffer:
                    text = buffer.decode('utf-8', errors='ignore').strip()
                    if text:
                        print(f"[{name}] {text}")
                    buffer = b''

            time.sleep(0.01)
        except Exception as e:
            if not stop_event.is_set():
                print(f"[{name}] Error: {e}")
            break

def main():
    print("=" * 50)
    print("Norr LoRa Bridge Test")
    print("=" * 50)

    # Check if ports exist
    try:
        ser1 = serial.Serial(PORT1, BAUD, timeout=0.1)
        ser2 = serial.Serial(PORT2, BAUD, timeout=0.1)
    except Exception as e:
        print(f"Error opening ports: {e}")
        print(f"Available ports: {PORT1}, {PORT2}")
        sys.exit(1)

    print(f"Opened {PORT1} (Device 1)")
    print(f"Opened {PORT2} (Device 2)")
    print()

    # Clear buffers
    ser1.reset_input_buffer()
    ser2.reset_input_buffer()
    time.sleep(0.5)

    # Start reader threads
    stop_event = threading.Event()
    received = []

    t1 = threading.Thread(target=read_serial, args=(ser1, "DEV1", stop_event, received))
    t2 = threading.Thread(target=read_serial, args=(ser2, "DEV2", stop_event, received))
    t1.daemon = True
    t2.daemon = True
    t1.start()
    t2.start()

    # Wait for any startup messages
    print("Waiting for devices to initialize...")
    time.sleep(2)

    # Send test message from Device 1 to Device 2
    test_msg = "Hello from Norr!"
    print(f"\nSending from Device 1: '{test_msg}'")

    frame = frame_message(test_msg)
    print(f"Frame: {frame.hex()}")
    ser1.write(frame)
    ser1.flush()

    # Wait for transmission and reception
    print("Waiting for LoRa transmission...")
    time.sleep(5)

    # Send from Device 2 to Device 1
    test_msg2 = "Reply from Device 2!"
    print(f"\nSending from Device 2: '{test_msg2}'")

    frame2 = frame_message(test_msg2)
    ser2.write(frame2)
    ser2.flush()

    # Wait for reception
    time.sleep(5)

    # Stop threads
    stop_event.set()
    time.sleep(0.5)

    # Close ports
    ser1.close()
    ser2.close()

    # Summary
    print()
    print("=" * 50)
    print("Test Summary")
    print("=" * 50)
    if len(received) >= 2:
        print("✓ SUCCESS: Bidirectional LoRa communication works!")
    elif len(received) == 1:
        print("⚠ PARTIAL: One direction works")
    else:
        print("✗ FAILED: No LoRa messages received")
        print("  Check: antenna connections, device distance, frequency settings")

    print(f"Messages received: {len(received)}")
    for msg in received:
        print(f"  - {msg}")

if __name__ == '__main__':
    main()
