AOU Darkroom Timer (v0.3)

This is the second development stage of my custom darkroom control system. It automates the exposure process for multigrade papers and drives an RGB LED panel (NeoPixel) as the light source for the enlarger.

The Origin Story: From Large Format Camera to a Closed-Loop System
The whole project actually started because I simply wanted a 4x5 enlarger. My basic idea: build a 16x16 NeoPixel matrix and mount it to my Sinar P so I could occasionally use the view camera as an enlarger.

To save expensive photo paper and time in the darkroom, I wanted to automate the exposure. Originally, I wrote the entire logic on an Arduino Mega using the Arduino IDE. It was fully functional, but quickly hit its limits as I kept adding more features. Because the complexity outgrew the hardware, I moved the project over to the much more capable ESP32-S3 architecture and PlatformIO. What started as a basic light source for my Sinar P has evolved into a complex custom device.

Current Status: The hardware architecture and core logic are rock solid. Right now, my main focus is on fleshing out the User Interface (UI) to make the system practical, fast, and smooth to use in an actual darkroom environment.
 How the System Works

The main difference compared to simple darkroom timers: The system doesn't just blindly count down seconds. It measures the actual amount of light (the dose) in real-time and adjusts the exposure dynamically. The setup relies on two entirely separate measuring loops:
1. Pre-Metering (Spot Meter / Densitometer)

Before exposing, the light on the baseboard (shadows/highlights) is measured.

    Hardware: A TSL2591 sensor. It's either connected via cable (I2C Bus 0) or swapped out for a wireless probe (ESP32-C3 + TSL2591) that broadcasts its readings 5 times a second via ESP-NOW.

    Purpose: Based on these readings, the timer calculates the required times for the soft and hard exposures, matching the selected paper profile. This sensor is also used for the built-in densitometer mode (measuring base fog and optical density log10).

2. Closed-Loop Exposure (Dose Mode)

During the exposure, the brightness of the 256 LEDs inevitably fluctuates as they heat up.

    Hardware: A second sensor (TSL2561) sits directly at the light head and is read via an exclusive, fast I2C bus (400 kHz).

    Purpose: In "Dose Mode", the ESP32 sums up the actual light output (Lux * Time). As soon as the calculated target light dose is reached, it shuts the light off. During the last 10% of the exposure, the panel is smoothly dimmed down to hit the target value with absolute precision.

(Technical Background: Updating a NeoPixel matrix temporarily blocks all hardware interrupts. To ensure the measuring sensor doesn't block the bus or miss readings during the exposure, it gets its own dedicated hardware I2C bus.)
 Core Features

    Multigrade Control: * Splitgrade Mode: Exposes separately with Green (Soft) and Blue (Hard).

        BW Mode (Mixed Light): Dynamically mixes the green and blue channels to achieve grades from 0.0 to 5.0 in a single exposure.

    Test Strip Generator: Automatically calculates exposure sequences (e.g., in 1/3 or 1/6 stop increments) and runs through them at the push of a button.

    Paper Profiles & Calibration: Stores up to 20 paper types in PSRAM/EEPROM. A built-in wizard guides you through taking measurements to determine the specific K-factors (sensitivity) of the paper.

    Burning / Dodging: Quick switching to burn-in mode with adjustable EV corrections.

    Temperature Compensation: A DS18B20 measures the LED heatsink temperature; in Dose Mode, this value is factored into the calculation to compensate for thermal drift.

 Operation & UI (In Active Development)

The base unit is operated blindly using three rotary encoders (for times and Grade/EV shifts) alongside a few tactile buttons. The feedback system includes:

    A Nextion Touch Display for navigating menus and switching modes.

    An RGB LCD hooked directly to the hardware providing color-coded feedback in the darkroom (Standby = Red, Exposure = Off, Error = Warning colors).

    A piezo buzzer for metronome ticks and haptic feedback.

 Software Architecture

    The main program uses PlatformIO and runs on an ESP32-S3 (N16R8). It relies heavily on external PSRAM (OPI) for large data structures (like the PaperBank) to keep the internal SRAM free for the NeoPixel timing and the FreeRTOS task handling the closed-loop control.

    The Wireless TSL2591 folder contains the firmware for the ESP32-C3/S3 wireless measuring probe.