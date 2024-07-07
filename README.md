# Autonomous Robot Vehicle

This repository contains the code for an autonomous robot vehicle designed to follow lines, avoid obstacles, and navigate its environment intelligently using various sensors and actuators. The project is structured to be modular, making it easy to extend and maintain.

## Table of Contents
- [Autonomous Robot Vehicle](#autonomous-robot-vehicle)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Features](#features)
  - [Hardware Requirements](#hardware-requirements)
  - [Software Requirements](#software-requirements)
  - [Installation](#installation)
  - [Usage](#usage)
  - [Code Structure](#code-structure)
  - [Settings Configuration](#settings-configuration)
- [Settings for Robot Functions](#settings-for-robot-functions)
- [Motor settings](#motor-settings)
- [Servo settings](#servo-settings)
- [Line following color selection (0 for white line, 1 for black line)](#line-following-color-selection-0-for-white-line-1-for-black-line)

## Introduction
The autonomous robot vehicle is designed to demonstrate the capabilities of modern robotics in terms of navigation and obstacle avoidance. It utilizes a combination of motor control, servo control, ultrasonic sensors, and LED indicators to move around its environment, detect and avoid obstacles, and follow predefined lines.

## Features
- **Line Following**: Uses infrared sensors to detect and follow lines on the ground.
- **Obstacle Avoidance**: Employs ultrasonic sensors to detect obstacles and navigate around them.
- **Dynamic Speed Control**: Adjusts speed based on proximity to obstacles.
- **LED Indicators**: Uses RGB LEDs to indicate the robot's status and actions.

## Hardware Requirements
- Raspberry Pi (any model with GPIO pins)
- Motor driver (e.g., L298N)
- Servo motors (e.g., SG90)
- Ultrasonic sensor (e.g., HC-SR04)
- Infrared sensors for line detection
- RGB LEDs
- Adafruit PCA9685 PWM Driver
- Power supply (battery pack)
- Connecting wires and a breadboard (optional for prototyping)

## Software Requirements
- Python 3.x
- RPi.GPIO
- Adafruit-PCA9685
- rpi_ws281x
- psutil

## Installation
1. **Clone the repository**:
    ```bash
    git clone https://github.com/bsharabi/CarSmartAI.git
    cd CarSmartAI
    ```

2. **Install the required Python packages**:
    ```bash
    sudo pip3 install -r requirements.txt
    ```

3. **Ensure you have the necessary permissions**:
    - The script may require root permissions to access GPIO pins, so you might need to run it with `sudo`.

## Usage
1. **Ensure all hardware components are properly connected**:
    - Follow the pin configuration as defined in the `settings.py` file.

2. **Run the main script**:
    ```bash
    sudo python3 AutonomousVehicle.py
    ```

3. **Stop the program**:
    - Press `Ctrl+C` to stop the program safely.

## Code Structure
- `AutonomousVehicle.py`: Main script for running the autonomous vehicle.
- `RobotFunctions.py`: Contains functions for line following.
- `RobotMove.py`: Controls the movement of the robot.
- `RobotLight.py`: Manages the RGB LEDs for status indication.
- `ServoCtrl.py`: Handles servo motor operations.
- `UltrasonicSensor.py`: Manages the ultrasonic sensor for obstacle detection.
- `settings.py`: Configuration file for defining GPIO pins and other settings.

## Settings Configuration
Make sure your `settings.py` file contains the following configurations:
```python
# Settings for Robot Functions
LINE_PIN_LEFT = 19
LINE_PIN_MIDDLE = 16
LINE_PIN_RIGHT = 20

# Motor settings
MOTOR_SPEED = 75

# Servo settings
ANGLE_RATE = 1

# Line following color selection (0 for white line, 1 for black line)
COLOR_SELECT = 1