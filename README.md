# STM32 DSP Lab

![Platform](https://img.shields.io/badge/platform-STM32-blue)
![Language](https://img.shields.io/badge/language-C-green)
![Toolchain](https://img.shields.io/badge/toolchain-arm--none--eabi--gcc-orange)
![Status](https://img.shields.io/badge/status-active-development-brightgreen)

Real-time digital signal processing experiments on STM32
microcontrollers.

------------------------------------------------------------------------

## Table of Contents

-   [Overview](#overview)
-   [Project Status](#project-status)
-   [Labs](#labs)
-   [Design Philosophy](#design-philosophy)
-   [Repository Structure](#repository-structure)
-   [Build & Flash](#build--flash)
-   [Measurement Philosophy](#measurement-philosophy)
-   [Roadmap](#roadmap)
-   [Vision](#vision)

------------------------------------------------------------------------

## Overview

STM32 DSP Lab is a structured collection of firmware experiments focused
on:

-   Deterministic timing behavior\
-   DMA-driven streaming architectures\
-   Real-time DSP under hardware constraints\
-   Measurable latency and throughput\
-   Fixed-point vs floating-point trade-offs

This project is not about abstract DSP correctness alone.\
It is about **what actually fits inside a real-time window on real
silicon**.

> DSP theory is only interesting if it survives contact with clock
> cycles.

------------------------------------------------------------------------

## Project Status

The deterministic streaming backbone is validated on STM32F2xx hardware.

Real-time FFT spectral analysis is operational and streaming live
magnitude data to a host PC via UART.

------------------------------------------------------------------------

## Labs

### Lab 1 -- Deterministic DMA Streaming Backbone

-   Timer-triggered sampling
-   Circular DMA buffer
-   Half-transfer synchronization
-   Lockstep main-loop processing
-   Deterministic execution guarantees

Documentation:\
[`labs/lab_1/README.md`](labs/lab_1/README.md)

------------------------------------------------------------------------

### Lab 2 -- Real-Time FFT Spectral Pipeline

-   CMSIS-DSP Q15 FFT
-   Magnitude spectrum computation
-   Structured binary UART protocol
-   Host-side live visualization (Python + Matplotlib)
-   Deterministic DSP insertion into ping-pong windows

Documentation:\
[`labs/lab_2/README.md`](labs/lab_2/README.md)

------------------------------------------------------------------------

## Design Philosophy

-   Hardware-timed pipelines over polling loops\
-   Deterministic execution over abstraction convenience\
-   Explicit buffer ownership and memory layout\
-   No heavy computation inside interrupts\
-   Measurable and reproducible experiments

Each lab extends a stable streaming foundation before adding algorithmic
complexity.

------------------------------------------------------------------------

## Repository Structure

    /core        -> Shared hardware abstraction utilities
    /dsp         -> DSP algorithms and experiments
    /drivers     -> Peripheral configuration (ADC, DMA, TIM, UART)
    /examples    -> Self-contained demo applications
    /benchmarks  -> Cycle counts and performance tests
    /docs        -> Notes and design documentation
    /labs        -> Structured lab exercises

Inside `/labs`:

    lab_1/
    lab_2/

------------------------------------------------------------------------

## Build & Flash

### Toolchain

-   ARM GNU Toolchain (`arm-none-eabi-gcc`)
-   Makefile-based builds
-   OpenOCD / ST-Link for flashing

Example:

``` bash
make
./scripts/openocd.sh
```

Each lab contains detailed instructions in its README.

------------------------------------------------------------------------

## Measurement Philosophy

All experiments must be:

-   Observable (UART output, FFT plots, buffer dumps)
-   Measurable (cycle counts, CPU load, memory usage)
-   Repeatable
-   Hardware-verified

Simulation is useful for reasoning.\
Hardware is final authority.

------------------------------------------------------------------------

## Roadmap

Planned topics:

-   FIR filters (fixed vs float comparison)
-   IIR quantization analysis
-   Windowing functions (Hann, Hamming, Blackman)
-   Log-magnitude (dB) scaling
-   Multi-stage DSP pipelines
-   Performance benchmarking across MCU families
-   CMSIS-DSP vs custom FFT comparison
-   Real ADC audio input pipeline
-   Real-time modulation chains

------------------------------------------------------------------------

## Vision

STM32 DSP Lab aims to evolve into a reusable deterministic real-time DSP
experimentation framework.

The goal is not just to implement DSP algorithms ---\
but to understand their cost in cycles, memory, and latency under strict
timing constraints.

Real-time is a contract.\
This repository measures whether that contract is upheld.
