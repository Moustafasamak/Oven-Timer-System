# Oven-Timer-System
Digital oven timer using PIC16F877A with keypad input, LCD display, status LEDs, and buzzer alarm.
# PIC16F877A Oven Timer

A digital oven timer system implemented using the **PIC16F877A microcontroller** operating at **4 MHz**.  
The project allows users to set a countdown time in **MM:SS format** using a keypad and control the timer with dedicated buttons.

## Features
- Time entry using a 3×4 keypad (minutes first, then seconds)
- Start, Pause, and Reset push buttons
- Real-time countdown displayed on a 16×2 LCD
- Visual status indication using LEDs and a bar-graph
- Audible buzzer alarm when the timer finishes
- Child-lock feature to prevent accidental input
- Accurate timing using Timer0 interrupt

## Hardware Components
- PIC16F877A microcontroller
- 4 MHz crystal oscillator
- 16×2 LCD (4-bit mode)
- 3×4 matrix keypad
- Push buttons (Start / Pause / Reset)
- LEDs (status + bar graph)
- Buzzer
- 5V regulated power supply

## Software Tools
- MPLAB X IDE
- XC8 Compiler
- PICkit Programmer

## How It Works
1. The user enters the desired time using the keypad (MM first, then SS).
2. Press **Start** to begin the countdown.
3. The remaining time is updated every second on the LCD.
4. When the timer reaches zero, LEDs and the buzzer are activated.
5. Press **Reset** to stop the alarm and return to the setup mode.

## Author
- **Moustafa Samak**
- **Asma Hasan**

