# Lab 1 -- Audio Stream Simulation with DMA and Circular Ping-Pong Buffer

## Objective

The goal of Lab 1 is to build a deterministic real-time audio stream
simulation on an STM32F207 microcontroller. This lab focuses on
establishing a clean and hardware-driven data pipeline using:

-   Timer-triggered DMA transfers
-   Circular (ping-pong) buffer architecture
-   Interrupt-based half/full transfer signaling
-   Parametric audio signal generation
-   DAC-based real-time audio output

No signal processing is performed in this lab. The objective is to
create a stable streaming foundation that will be extended in Lab 2.

------------------------------------------------------------------------

## System Architecture

The system simulates an audio acquisition and playback pipeline using
hardware peripherals and DMA.

### 1. Audio Signal Generation

A parametric audio signal (multi-tone DDS-based generator with optional
noise) is generated in software.\
The generator operates at a fixed sample rate (e.g., 44.1 kHz).

### 2. Timer-Driven Sample Clock

TIM2 is configured to run at the selected sample rate.\
Each timer update event generates a DMA request.

This ensures that the system timing is hardware-driven and
deterministic.

### 3. DMA Circular Buffer (Ping-Pong)

A circular buffer of size **N** samples is used:

```
|----------- N samples -----------|
|------ N/2 ------|------ N/2 ----|
```

DMA behavior:

-   On each timer event, one sample is written into the buffer.
-   After **N/2 samples**, a Half Transfer (HT) interrupt is triggered.
-   After **N samples**, a Transfer Complete (TC) interrupt is
    triggered.
-   The buffer then wraps automatically (circular mode).

This creates a classic ping-pong buffer structure.

### 4. CPU Notification via Interrupts

-   HT Interrupt → First half of buffer is ready for processing.
-   TC Interrupt → Second half of buffer is ready for processing.

The CPU can safely operate on one half of the buffer while DMA fills the
other half.

In Lab 1, no processing is performed. The interrupts are used only to
validate correct timing and buffer operation.

### 5. DAC Output (Test Output Stage)

For validation purposes, the generated audio signal is routed to the
onboard DAC.

This allows:

-   Listening to the generated waveform
-   Verifying timing stability
-   Confirming correct DMA-to-peripheral operation

### 6. Continuous FFT Monitoring (Visualization Path)

In parallel, an FFT runs continuously on the buffered data.\
The resulting spectrum is transmitted via UART to a PC application.

The GUI displays:

-   Real-time frequency spectrum
-   System activity visibility
-   Confirmation of correct signal generation

This provides insight into what the processor is handling internally.

------------------------------------------------------------------------

## Lab 1 Scope

Lab 1 includes:

-   Hardware timer configuration
-   DMA setup in circular mode
-   HT/TC interrupt handling
-   Ping-pong buffer management
-   Parametric signal generation
-   DAC-based audio output
-   UART-based FFT data transmission

Lab 1 explicitly excludes:

-   Any audio processing or filtering
-   DSP effects
-   Buffer modifications beyond basic streaming

The focus is purely on building a robust real-time streaming
infrastructure.

------------------------------------------------------------------------

## What You Should Observe

-   Stable HT and TC interrupt cadence
-   Continuous circular buffer operation
-   Audible tone output via DAC
-   Real-time FFT visualization on PC
-   No buffer overruns or timing instability

------------------------------------------------------------------------

## Why This Architecture Matters

This structure mirrors professional embedded audio systems:

-   Hardware-timed sample clocks
-   DMA-based data movement
-   Ping-pong buffering for deterministic processing windows

It provides a clean separation between:

-   Data acquisition
-   Data transport
-   Processing
-   Output

In Lab 2, actual signal processing will be inserted into the processing
window created by the ping-pong buffer structure.

------------------------------------------------------------------------

## Next Steps (Lab 2 Preview)

In Lab 2:

-   Real-time DSP processing will be added
-   Processing will occur inside the HT/TC windows
-   Latency and computational limits will be analyzed

Lab 1 ensures that the streaming backbone is stable before adding
computational complexity.
