#!/usr/bin/env python3
"""
plot_fft_uart.py
================

Live plot of an FFT magnitude spectrum streamed from an STM32 over UART.

Expected frame format (little-endian)
-------------------------------------

    SYNC          : 2  bytes   0xA5 0x5A
    frame_counter : 4  bytes   uint32 (monotonic counter)
    count         : 2  bytes   uint16 (number of magnitude values)
    payload       : 2*count    int16[count] (e.g. Q15 magnitudes)
    checksum      : optional   (ignored in this script)

This script implements a robust stream parser:
- Continuously scans for SYNC
- Validates header fields
- Waits until a complete frame is available
- Extracts and returns the payload as NumPy array

Matplotlib
----------
TkAgg backend is used for interactive plotting.
`plt.pause(...)` keeps the GUI responsive without blocking.

Adjust in main():
- PORT / BAUD
- FS (sampling rate)
- N  (FFT length used on the MCU)
"""

from __future__ import annotations

import struct
import time
from typing import Optional, Tuple

import numpy as np
import serial

import matplotlib
matplotlib.use("TkAgg")  # interactive backend
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

SYNC = b"\xA5\x5A"                 # 2-byte synchronization word
HEADER_FMT = "<IH"                 # uint32 frame_counter, uint16 count
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 6 bytes
SYNC_SIZE = 2

# Safety limit to avoid allocating absurd buffers if the stream is corrupted
MAX_ALLOWED_BINS = 4096


# ---------------------------------------------------------------------------
# Frame reader
# ---------------------------------------------------------------------------

class FrameReader:
    """
    Incremental frame parser for a continuous UART byte stream.

    The class maintains an internal byte buffer.
    New data is appended via `feed()`.
    `next_frame()` attempts to extract one complete frame.

    Returns:
        (frame_counter, magnitudes) if a complete frame is available
        None otherwise
    """

    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, chunk: bytes) -> None:
        """Append newly received UART bytes to the internal buffer."""
        if chunk:
            self.buf += chunk

    def next_frame(self) -> Optional[Tuple[int, np.ndarray]]:
        """
        Attempt to extract one frame from the buffer.

        The function:
        1. Searches for SYNC
        2. Validates header
        3. Waits for full payload
        4. Returns parsed data
        """

        while True:
            # Search for synchronization marker
            i = self.buf.find(SYNC)
            if i < 0:
                # Keep at most the last byte to avoid unlimited growth
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]
                return None

            # Discard garbage before SYNC
            if i > 0:
                del self.buf[:i]

            # Ensure header is fully available
            if len(self.buf) < SYNC_SIZE + HEADER_SIZE:
                return None

            # Parse header fields
            frame_counter, count = struct.unpack_from(
                HEADER_FMT,
                self.buf,
                SYNC_SIZE
            )

            # Basic sanity checks
            if count == 0 or count > MAX_ALLOWED_BINS:
                # Invalid header → shift by one byte and retry
                del self.buf[:1]
                continue

            payload_len = 2 * count
            total_len = SYNC_SIZE + HEADER_SIZE + payload_len

            # Wait for full frame
            if len(self.buf) < total_len:
                return None

            # Extract payload
            payload = bytes(
                self.buf[SYNC_SIZE + HEADER_SIZE :
                         SYNC_SIZE + HEADER_SIZE + payload_len]
            )

            # Interpret payload as int16 magnitudes
            mags = np.frombuffer(payload, dtype=np.int16).astype(np.float32)

            # Remove processed frame from buffer
            del self.buf[:total_len]

            return frame_counter, mags


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------

def main() -> None:
    """
    Open serial port, parse frames, and display live FFT spectrum.
    """

    # --- Serial configuration ---
    PORT = "/dev/ttyACM0"
    BAUD = 115200

    ser = serial.Serial(PORT, baudrate=BAUD, timeout=0.01)
    time.sleep(0.2)
    ser.reset_input_buffer()

    # --- FFT parameters (must match MCU configuration) ---
    FS = 44100.0       # sampling rate [Hz]
    N = 256            # FFT length on MCU
    bins_expected = N // 2 + 1

    # Frequency axis for plotting
    f = np.arange(bins_expected) * FS / N

    fr = FrameReader()

    # --- Plot setup ---
    plt.figure("Live Spectrum (UART)")
    line, = plt.plot(f, np.zeros_like(f))
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude (Q15 → float)")
    plt.title("Live Spectrum (UART)")
    plt.grid(True)
    plt.ylim(0, 1.0)      # fixed y-range reduces flicker
    plt.tight_layout()
    plt.show(block=False)

    last_frame_counter = None
    frames = 0
    t_stat = time.time()

    # --- Main loop ---
    while True:
        # Read available UART data
        chunk = ser.read(4096)
        if chunk:
            fr.feed(chunk)

        out = fr.next_frame()
        if out is None:
            # Keep GUI responsive
            plt.pause(0.001)
            continue

        frame_counter, mags = out

        # Ensure expected FFT size
        if len(mags) != bins_expected:
            continue

        # Convert Q15 to normalized float magnitude
        mags = np.abs(mags / 32768.0)

        # Update plot
        line.set_ydata(mags)

        if frame_counter != last_frame_counter:
            plt.pause(0.001)
            last_frame_counter = frame_counter

        # Simple FPS statistics
        frames += 1
        if time.time() - t_stat > 2.0:
            print(f"fps ~ {frames/2.0:.1f}, buffered={len(fr.buf)}")
            frames = 0
            t_stat = time.time()


if __name__ == "__main__":
    main()