# CRYSTALS-Kyber Clean and M4 Optimization Implementations

## Overview
This project provides implementations of the **CRYSTALS-Kyber** post-quantum cryptography algorithm, focusing on both clean and M4-optimized versions. The algorithms have been benchmarked on an STM Nucleo-F446RE development board with a focus on performance metrics such as clock cycles during key generation, encryption, and decryption processes.

---

## Test Environment

### Hardware: STM Nucleo-F446RE
The STM Nucleo-F446RE board features:
- **Processor:** ARM Cortex-M4 32-bit CPU with an FPU, clocked up to 180 MHz.
- **Memory:**
  - 512 KB Flash memory
  - 128 KB + 4 KB SRAM
- **Interfaces:**
  - 20 communication interfaces, including USART/UART, SPI, I2C, CAN, and SDIO
  - Parallel camera interface (8-14 bit) with up to 54 MB/s bandwidth
- **Power Features:**
  - Operates at 1.7 V to 3.6 V
  - Low-power modes: Sleep, Stop, Standby
  - VBAT supply for RTC and backup registers

### Software: STM32CubeIDE
The project was developed and tested using **STM32CubeIDE**, an integrated development environment designed for STM32 microcontrollers. Key features of the environment:
- Based on the Eclipse CDT framework.
- Utilizes the GCC toolchain for compilation and GDB for debugging.
- Arm GNU Toolchain for final testing and benchmarking.

---

## Project Structure

The following projects were created within STM32CubeIDE, corresponding to different parameter sets and optimizations of the Kyber algorithm:
- `kyber512-clean`
- `kyber512-m4`
- `kyber768-clean`
- `kyber768-m4`
- `kyber1024-clean`
- `kyber1024-m4`

Each project implements:
- **Key Generation**
- **Encryption**
- **Decryption**

The projects utilize the CRYSTALS-Kyber implementations from the third round of the [NIST PQC competition](https://csrc.nist.gov/Projects/post-quantum-cryptography/round-3-submissions) and supplementary code for:
- Random byte generation
- FIPS202 hash function

---

## Execution

### Test Procedure
- Each of the three operations—Key Generation, Encryption, and Decryption—was executed 40 times for each parameter set.
- The number of clock cycles for each operation was measured and averaged.

---

## Repository Structure
The full source code for all implementations is available in this repository:
- Clean implementations (`-clean` suffix): Standard, unoptimized implementations.
- M4-optimized implementations (`-m4` suffix): Optimized for the ARM Cortex-M4 core.