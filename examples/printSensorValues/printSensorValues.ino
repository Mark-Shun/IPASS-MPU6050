#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.hpp"

mpu6050 mySensor;
CalcSensorData calc_data;

unsigned long int timer = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  byte status;
  status = mySensor.wakeUp();
  if(status != 0){
    while(status != 0){
      Serial.print("MPU6050 Initialization failed, error: "); Serial.println(status);
      status = mySensor.wakeUp();
      delay(50);
    }
  }
  mySensor.setClockSource(0x01);
  mySensor.setSampleRate(0x00);
  mySensor.setFSYNC(0x00); // Input disabled
  mySensor.setDLPF(0x00); // Bandwith 260Hz delay = 0ms
  mySensor.setGyroFullScaleRange(1); // ±500°/s and the sensitivity is 65.5 LSB per °/s
  mySensor.setAccelFullScaleRange(0); 
  mySensor.calcOffsets();
}

void loop() {
  calc_data = mySensor.getAllCalcSensorData();
  if((millis()-timer)>100){ // print data every 100ms
    Serial.print("AccelX: "); Serial.print(calc_data.calc_accel_x);
    Serial.print("\tAccelY: "); Serial.print(calc_data.calc_accel_y);
    Serial.print("\tAccelZ: "); Serial.println(calc_data.calc_accel_z);
    Serial.print("Temperature: "); Serial.print(calc_data.calc_temperature); Serial.println(" celcius");
    Serial.print("GyroX: "); Serial.print(calc_data.calc_gyro_x);
    Serial.print("\tGyroY: "); Serial.print(calc_data.calc_gyro_y);
    Serial.print("\tGyroZ: "); Serial.println(calc_data.calc_gyro_z);
    timer = millis();
  }

}