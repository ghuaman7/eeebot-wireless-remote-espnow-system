# EEEBot Wireless Remote-Control System (ESP-NOW, ESP32)

This repository contains the code and design for a wireless remote-control system developed for the EEEBot, a small autonomous robot built as part of an engineering project. The system is made up of two ESP32 microcontrollers communicating through ESP-NOW, which allows them to send and receive data directly without using Wi-Fi. The aim was to create a remote interface that supports both real-time manual control using a joystick and programmable command sequences through a numpad and LCD.

Both systems are included here:
- Joystick Mode: manually drives the robot in real time
- LOGO Mode: sends a sequence of movements and rotations for the robot to follow automatically

Each mode is split into a remote (master) and a robot (slave) side.

# Overview of the System

The remote ESP32 collects user input (either via joystick or keypad), sends instructions wirelessly using ESP-NOW, and gives visual and sound feedback using an LCD screen, LED, and buzzer. The robot ESP32 reads these instructions and controls the motors accordingly. It also uses an ultrasonic sensor to detect nearby obstacles and alerts the user if something is too close.

ESP-NOW was chosen over Wi-Fi because it's faster and doesn’t require a network connection, which makes it better for real-time tasks like maze navigation.


## What I Used (Component List)

This is a rough summary of the hardware used to build both the remote and robot:

- **ESP32 microcontrollers (2)** – one for the remote, one on the robot
- **Joystick (KY-023)** – used in manual control mode
- **3x4 Matrix Numpad** – used in LOGO command mode for user input
- **16x2 LCD Display** – shows the entered commands in real time
- **Piezo Buzzer** – gives audio alerts if an obstacle is detected
- **LED** – used as a visual warning system
- **HC-SR04 Ultrasonic Sensor** – measures distance to obstacles
- **Stripboard and jumper wires** – for building the remote circuit
- **Resistors and headers** – basic circuit components to protect outputs and connect modules

## How It Works

**Joystick Mode**:  
- Joystick values are read with `analogRead()`
- These values are mapped to speed/direction and sent over ESP-NOW
- The robot receives the values, converts them into PWM motor commands
- If something is detected closer than 10 cm, the robot sends an alert back
- The remote buzzes and flashes the LED to warn the user

**LOGO Mode**:  
- The user enters commands like `FORWARD 3`, `RIGHT 90` using the numpad
- Commands are displayed on the LCD and saved in memory
- When confirmed with `#`, the full set is sent over ESP-NOW
- The robot executes each command in order

## Technical Notes

- The code uses `LiquidCrystal.h` for LCD and `Keypad.h` for the matrix keypad
- All communication is done using `esp_now.h`, without Wi-Fi setup
- Some smoothing was applied to joystick inputs, but this also introduced a bit of lag
- The HC-SR04 works well for detecting nearby obstacles but the buzzer can be a bit annoying in quiet spaces (next time I'd make it pulse or beep instead of a constant tone)

## Reflections and Lessons Learned

This was a big step for me in building a fully wireless system that didn’t rely on Wi-Fi. I learned a lot about prototyping under physical space constraints (stripboards get crowded fast), calibrating sensors like joysticks and ultrasonic modules, and making feedback intuitive for users. ESP-NOW worked really well and I’d use it again, but I’d definitely redesign the layout to make it easier to handle and more reliable. Also, using encoders instead of timing-based movement would make the navigation much more accurate.

## About

This project was developed as part of my university coursework for Electrical and Electronic Engineering. The work was split into two parts (manual control and programmed instructions), both targeting autonomous maze navigation using the same hardware system.

Author: Genesis Huaman
Institution: [University of Nottingham]

## License

This project is licensed under the MIT License. See the LICENSE file for more details.



