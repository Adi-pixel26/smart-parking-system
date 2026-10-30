# Smart Parking System (STM32F411 Bare-Metal)

Bare-metal embedded C smart parking system implemented on the **STM32F411 (Black Pill)** microcontroller using STM32CubeIDE.

---

## Features

- **Dual HC-SR04 Ultrasonic Sensors:**
  - Entry gate sensor (`PA0` Trig, `PA1` Echo via `EXTI1` Hardware Interrupt).
  - Exit gate sensor (`PA2` Trig, `PA3` Echo via `EXTI3` Hardware Interrupt).
  - High-precision microsecond pulse width timing using 32-bit hardware timer `TIM2`.
  - Non-blocking alternating ping schedule with lost-pulse timeout recovery.

- **Dual SG90 Servo Motors (Barrier Gates):**
  - Entry gate barrier (`PA6` via `TIM3_CH1` PWM, 50 Hz).
  - Exit gate barrier (`PA7` via `TIM3_CH2` PWM, 50 Hz).
  - Hysteresis and debounce logic (< 10 cm to open, > 15 cm clear confirmation) to prevent motor chatter.

- **16x2 I2C Character LCD (JHD162A / PCF8574T):**
  - Hardware I2C1 communication (`PB6` SCL, `PB7` SDA).
  - 4-bit nibble transmission with bus recovery and address auto-detection (`0x27` / `0x3F`).
  - Real-time display showing parking slot status and warnings (`PARKING FULL!`).

- **Slot Management:**
  - Dynamic slot tracking (`MAX_SLOTS = 5`).
  - Automatic single-decrement on vehicle entry and single-increment on vehicle exit.

---

## Hardware Pinout Mapping

| Component | Signal | STM32F411 Pin | Mode | Description |
| :--- | :--- | :--- | :--- | :--- |
| **I2C LCD** | SCL | **PB6** | AF4 (I2C1) | Serial Clock |
| | SDA | **PB7** | AF4 (I2C1) | Serial Data |
| | VCC / GND | **5V / GND** | Power | 5V Required for LCD Crystals |
| **Entry Sensor (HC-SR04)** | Trig | **PA0** | GPIO Out | 10 µs Trigger Pulse |
| | Echo | **PA1** | EXTI1 | Echo Pulse Duration |
| **Exit Sensor (HC-SR04)** | Trig | **PA2** | GPIO Out | 10 µs Trigger Pulse |
| | Echo | **PA3** | EXTI3 | Echo Pulse Duration |
| **Entry Servo (Gate 1)** | PWM | **PA6** | AF2 (TIM3_CH1) | 50 Hz Servo Pulse |
| **Exit Servo (Gate 2)** | PWM | **PA7** | AF2 (TIM3_CH2) | 50 Hz Servo Pulse |

---

## Toolchain & Building

- **IDE:** STM32CubeIDE 1.19.0
- **Compiler:** `arm-none-eabi-gcc` (13.3.rel1)
- **Target MCU:** STM32F411CEU6 (Arm Cortex-M4 @ 16 MHz HSI)
