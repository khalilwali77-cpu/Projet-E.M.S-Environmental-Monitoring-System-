EMS System – Smart Environmental Monitoring System
Description

EMS (Environmental Monitoring System) is an embedded system project developed using STM32L475.
The system monitors temperature and gas concentration and automatically reacts using LEDs, a buzzer, and a ventilation system.

This project was implemented in C using STM32CubeIDE and HAL drivers.

Features

Push-button ON/OFF control

Boot message display on LCD

Real-time temperature monitoring (HTS221 – I2C)

Gas detection using MQ-2 sensor (ADC)

Adjustable temperature threshold

Adjustable gas threshold

LCD 16x2 (I2C) user interface

Finite State Machine (FSM) architecture

Three alarm modes:

Temperature alarm

Gas alarm

Combined temperature and gas alarm

Non-blocking buzzer control

Hysteresis to prevent oscillation around thresholds

System Architecture

The system is based on a Finite State Machine with the following states:

ST_OFF

ST_BOOT

ST_SHOW

ST_SET_TEMP

ST_SET_GAS

ST_NORMAL

ST_AL_TEMP

ST_AL_GAS

ST_AL_BOTH

This architecture ensures structured logic, scalability, and real-time responsiveness.

Hardware Components

STM32L475VG (B-L475E-IOT01A)

HTS221 Temperature Sensor (I2C2)

MQ-2 Gas Sensor (ADC1)

16x2 LCD with I2C interface

Push buttons (ON/OFF, +, −, OK)

LEDs (Green, Yellow, Red, Blue)

Buzzer

DC fan controlled via MOSFET

Software Details

Programming language: C

IDE: STM32CubeIDE

Drivers: STM32 HAL

Communication protocols:

I2C (LCD and temperature sensor)

ADC (gas sensor)

Timing management using HAL_GetTick()

Future Improvements

Bluetooth (BLE) integration

Mobile application control

Data logging functionality

Remote monitoring via WiFi
