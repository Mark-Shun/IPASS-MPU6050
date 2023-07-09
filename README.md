![MPU6050 Image](./documentation/html/MPU6050%20picture.png?raw=true "Title")
# IPASS MPU6050 Library
This repository contains a MPU6050 library written for the course IPASS.
It uses the Wire library to interface over I2C between the master device and the module. 

The examples folder contains an Arduino example file that prints converted values (accelerometer = g-force unit, gyro = degrees per second, temperature = Celsius) calculated from the raw module values.

The test_MPU6050 contains test functions in the Unity test framework.

This library is licensed under the boost license.