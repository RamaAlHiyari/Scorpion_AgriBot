# Scorpion_AgriBot
Scorpion AgriBot is an Arduino Mega–based rover that autonomously explores crop rows, measures soil moisture via a servo-mounted probe, and avoids obstacles with ultrasonic sensing. Optional ArduCAM captures leaf images, while all readings (time, moisture %, distance, notes, image) are logged as CSV to microSD for offline analysis.
# Scorpion AgriBot – Soil & Plant Exploration Robot

**Robotics | Arduino Mega | 4× DC Motors | Soil Moisture | Ultrasonic | Servo Arm | (Optional) ArduCAM + SD**

## Overview
Scorpion AgriBot is a mobile agricultural rover designed to explore soil conditions in structured farms and diagnose them via a soil-moisture probe, with support for capturing plant-leaf photos using an ArduCAM/webcam for documentation and later analysis. The system uses an Arduino Mega 2560 to drive four DC motors through L298N drivers, avoids obstacles with an ultrasonic sensor, and uses a servo arm to lower and raise the moisture probe for periodic sampling, logging results to CSV on an SD card.

**Goal:** Survey crop fields, measure soil moisture at multiple points, capture leaf images, and record all data for diagnosis and follow-up.

## Features
- Autonomous navigation with obstacle avoidance using HC-SR04
- Servo arm to deploy the moisture probe, take a precise reading, then retract
- Data logging of time, moisture percentage, raw reading, distance, notes, and image name to CSV on SD
- Optional leaf imaging with ArduCAM OV2640 saved as JPEG
- Easy tuning of moisture thresholds (DRY/OK/WET), servo angles, and drive speeds

## Bill of Materials
- 1× Arduino Mega 2560  
- 2× L298N (or equivalent 4-channel motor drivers)  
- 4× DC motors with wheels  
- 1× HC-SR04 ultrasonic sensor (front)  
- 1× Analog soil-moisture probe  
- 1× SG90/MG90S servo for the probe arm  
- 1× microSD reader (optional but recommended for logging)  
- 1× ArduCAM Mini OV2640 (optional for imaging)  
- Separate power sources for motors and logic, with a common ground

## Quick Wiring (Arduino Mega)

### Motors (L298N)

**Left side**
- ENA → D5 (PWM), IN1 → D22, IN2 → D23  
- ENB → D6 (PWM), IN3 → D24, IN4 → D25  

**Right side**
- ENC → D9 (PWM), IN5 → D26, IN6 → D27  
- END → D10 (PWM), IN7 → D28, IN8 → D29  

### Sensors and Actuators
- Ultrasonic: TRIG → D31, ECHO → D30  
- Servo (probe arm): SIG → D11  
- Soil moisture (analog): AO → A0

### Storage and Camera (optional)
- SD: CS → D53 (SPI HW: MISO 50, MOSI 51, SCK 52)  
- ArduCAM OV2640: CS → D7 (shares SPI)

**Power note:** Power the motors from a separate battery through the L298N and share ground with the Arduino board.

## Images
https://drive.google.com/drive/folders/1-yvlwI3CvnW-3ASu3sCr5k0THxUEndf9?usp=sharing

