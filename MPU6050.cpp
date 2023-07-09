#include "MPU6050.hpp"

mpu6050::mpu6050(uint8_t set_i2c_address){
    this->i2c_address = set_i2c_address;
}

byte mpu6050::wakeUp(){
    byte status = 0;
    Wire.beginTransmission(i2c_address);
    Wire.write(power_management1);
    Wire.write(0);
    status = Wire.endTransmission();
    return status;
}

bool mpu6050::awakeCheck(){
    Wire.beginTransmission(i2c_address);
    Wire.write(0x6B);  // PWR_MGMT_1 register address
    Wire.endTransmission();

    Wire.requestFrom(i2c_address, static_cast<uint8_t>(1));  // Read 1 byte from MPU6050

    if (Wire.available()) {
    uint8_t powerManagement = Wire.read();
    // Check the sleep bit
    return ((powerManagement & 0x40) == 0);
    }
    // Return false if reading fails
    return false; 
}

byte mpu6050::communicationCheck(){
    byte status;
    Wire.beginTransmission(i2c_address);
    status = Wire.endTransmission();
    return status;
}

uint8_t mpu6050::whoAmI(){
    Wire.beginTransmission(i2c_address);
    Wire.write(who_am_i);
    Wire.endTransmission();

    Wire.requestFrom(i2c_address,static_cast<uint8_t>(1)); // Read one byte from MPU6050

    if(Wire.available()){
        uint8_t address = Wire.read();
        return address;
    }
    else{
        return 0;
    }
}

void mpu6050::reset(){
    Wire.beginTransmission(i2c_address);
    Wire.write(power_management1);
    Wire.endTransmission(false);

    Wire.requestFrom(i2c_address, static_cast<uint8_t>(1));
    uint8_t current_bits = Wire.read();

    // Set highest bit (reset) to 1 in register
    uint8_t new_bits = current_bits | 0x80;

    Wire.beginTransmission(i2c_address);
    Wire.write(power_management1);
    Wire.write(new_bits);
    Wire.endTransmission();
    
}

void mpu6050::calcOffsets(bool is_calc_acc, bool is_calc_gyro){
  if(is_calc_acc){ accel_x_offset = 0; accel_y_offset = 0; accel_z_offset = 0; }
  if(is_calc_gyro){ gyro_x_offset = 0; gyro_y_offset = 0; gyro_z_offset = 0; }
  float ag[6] = {0,0,0,0,0,0}; // 3 accel and 3 gyro values
  for(int i = 0; i < calib_offset_amount; i++){
    if(is_calc_acc){
	    ag[0] += this->getCalcAccelX();
	    ag[1] += this->getCalcAccelY();
	    ag[2] += ((this->getCalcAccelZ())-1.0);
    }
    if(is_calc_gyro){
	    ag[3] += this->getCalcGyroX();
	    ag[4] += this->getCalcGyroY();
	    ag[5] += this->getCalcGyroZ();
    }
	delay(1); // wait a little bit between 2 measurements
  }
  
  if(is_calc_acc){
    accel_x_offset = ag[0] / calib_offset_amount;
    accel_y_offset = ag[1] / calib_offset_amount;
    accel_z_offset = ag[2] / calib_offset_amount;
  }
  
  if(is_calc_gyro){
    gyro_x_offset = ag[3] / calib_offset_amount;
    gyro_y_offset = ag[4] / calib_offset_amount;
    gyro_z_offset = ag[5] / calib_offset_amount;
  }
}

void mpu6050::enableAccelSelfTest(){
    // Has to be set to +-8g mode
    setAccelFullScaleRange(0x02);
    Wire.beginTransmission(i2c_address);
    Wire.write(accel_config);
    // Send repeated starts
    Wire.endTransmission(false);

    Wire.requestFrom(i2c_address, static_cast<uint8_t>(1));
    uint8_t current_accel_config = Wire.read();

    // Enable self test bits
    uint8_t new_accel_config = current_accel_config |= 0xE0; // Self test for X, Y, Z gyro axes

    Wire.beginTransmission(i2c_address);
    Wire.write(accel_config);
    Wire.write(new_accel_config);
    Wire.endTransmission();
}

void mpu6050::enableGyroSelfTest(){
    // Has to be set to +-250dps mode
    setGyroFullScaleRange(0x00);
    Wire.beginTransmission(i2c_address);
    Wire.write(gyro_config);
    // Send repeated starts
    Wire.endTransmission(false);

    Wire.requestFrom(i2c_address, static_cast<uint8_t>(1));
    uint8_t current_gyro_config = Wire.read();

    // Enable self test bits
    uint8_t new_gyro_config = current_gyro_config |= 0xE0; // Self test for X, Y, Z gyro axes

    Wire.beginTransmission(i2c_address);
    Wire.write(gyro_config);
    Wire.write(new_gyro_config);
    Wire.endTransmission();
}

uint8_t mpu6050::getClockSource(){
    Wire.beginTransmission(i2c_address);
    Wire.write(power_management1);
    Wire.endTransmission();

    Wire.requestFrom(i2c_address, static_cast<uint8_t>(1));
    if(Wire.available()){
        uint8_t raw_setting = Wire.read();
        uint8_t mask = 0x07; // 00000111
        return (raw_setting & mask);
    }
    else{
        return 0;
    }
}

void mpu6050::getSelfTestValues(uint8_t & accel_x, uint8_t & accel_y, uint8_t & accel_z, uint8_t & gyro_x, uint8_t & gyro_y, uint8_t & gyro_z){
    Wire.beginTransmission(i2c_address);
    // Start extracting from SELF_TEST_X register
    Wire.write(0x0D);
    Wire.endTransmission(false);
    Wire.requestFrom(i2c_address, static_cast<uint8_t>(4));

    uint8_t self_test_x = Wire.read();
    uint8_t self_test_y = Wire.read();
    uint8_t self_test_z = Wire.read();
    uint8_t self_test_a = Wire.read();

    accel_x = (self_test_x >> 3) | (self_test_a & 0x30) >> 4;
    accel_y = (self_test_y >> 3) | (self_test_a & 0x0C) >> 2;
    accel_z = (self_test_x >> 3) | (self_test_a & 0x03);

    gyro_x = self_test_x & 0x1F;
    gyro_y = self_test_y & 0x1F;
    gyro_z = self_test_z & 0x1F;
}

int16_t mpu6050::getAccelX(){
    Wire.beginTransmission(i2c_address);
    Wire.write(accel_x_h);
    Wire.endTransmission();

    Wire.requestFrom(i2c_address,static_cast<uint8_t>(2)); //Retrieve 2 bytes from MPU6050

    if(Wire.available() >= 2){
        uint8_t first_byte = Wire.read();
        uint8_t second_byte = Wire.read();
        int16_t accel_x_value = (first_byte << 8) | second_byte;
        return accel_x_value;
    }
    else{
        return 0;
    }
}

float mpu6050::getCalcAccelX(){
    int16_t raw_data = this->getAccelX();
    return ((float)raw_data) / accel_lsb_to_g - accel_x_offset;
}

int16_t mpu6050::getAccelY(){
    Wire.beginTransmission(i2c_address);
    Wire.write(accel_y_h);
    Wire.endTransmission();

    Wire.requestFrom(i2c_address,static_cast<uint8_t>(2)); //Retrieve 2 bytes from MPU6050

    if(Wire.available() >= 2){
        uint8_t first_byte = Wire.read();
        uint8_t second_byte = Wire.read();
        int16_t accel_y_value = (first_byte << 8) | second_byte;
        return accel_y_value;
    }
    else{
        return 0;
    }
}

float mpu6050::getCalcAccelY(){
    int16_t raw_data = this->getAccelY();
    return ((float)raw_data) / accel_lsb_to_g - accel_y_offset;
}

int16_t mpu6050::getAccelZ(){
    Wire.beginTransmission(i2c_address);
    Wire.write(accel_z_h);
    Wire.endTransmission();

    Wire.requestFrom(i2c_address,static_cast<uint8_t>(2)); //Retrieve 2 bytes from MPU6050

    if(Wire.available() >= 2){
        uint8_t first_byte = Wire.read();
        uint8_t second_byte = Wire.read();
        int16_t accel_z_value = (first_byte << 8) | second_byte;
        return accel_z_value;
    }
    else{
        return 0;
    }
}

float mpu6050::getCalcAccelZ(){
    int16_t raw_data = this->getAccelZ();
    return (!upside_down_mount - upside_down_mount) * ((float)raw_data) / accel_lsb_to_g - accel_z_offset;
}

int16_t mpu6050::getGyroX(){
    Wire.beginTransmission(i2c_address);
    Wire.write(gyro_x_h);
    Wire.endTransmission();

    Wire.requestFrom(i2c_address,static_cast<uint8_t>(2)); //Retrieve 2 bytes from MPU6050

    if(Wire.available() >= 2){
        uint8_t first_byte = Wire.read();
        uint8_t second_byte = Wire.read();
        int16_t gyro_x_value = (first_byte << 8) | second_byte;
        return gyro_x_value;
    }
    else{
        return 0;
    }
}

float mpu6050::getCalcGyroX(){
    int16_t raw_data = this->getGyroX();
    return ((float)raw_data) / gyro_lsb_to_degsec - gyro_x_offset;
}

int16_t mpu6050::getGyroY(){
    Wire.beginTransmission(i2c_address);
    Wire.write(gyro_y_h);
    Wire.endTransmission();

    Wire.requestFrom(i2c_address,static_cast<uint8_t>(2)); //Retrieve 2 bytes from MPU6050

    if(Wire.available() >= 2){
        uint8_t first_byte = Wire.read();
        uint8_t second_byte = Wire.read();
        int16_t gyro_y_value = (first_byte << 8) | second_byte;
        return gyro_y_value;
    }
    else{
        return 0;
    }
}

float mpu6050::getCalcGyroY(){
    int16_t raw_data = this->getGyroY();
    return ((float)raw_data) / gyro_lsb_to_degsec - gyro_y_offset;
}

int16_t mpu6050::getGyroZ(){
    Wire.beginTransmission(i2c_address);
    Wire.write(gyro_z_h);
    Wire.endTransmission();

    Wire.requestFrom(i2c_address,static_cast<uint8_t>(2)); //Retrieve 2 bytes from MPU6050

    if(Wire.available() >= 2){
        uint8_t first_byte = Wire.read();
        uint8_t second_byte = Wire.read();
        int16_t gyro_z_value = (first_byte << 8) | second_byte;
        return gyro_z_value;
    }
    else{
        return 0;
    }
}

float mpu6050::getCalcGyroZ(){
    int16_t raw_data = this->getGyroZ();
    return ((float)raw_data) / gyro_lsb_to_degsec - gyro_z_offset;
}

int16_t mpu6050::getTemp(){
    Wire.beginTransmission(i2c_address);
    Wire.write(temperature);
    Wire.endTransmission();

    Wire.requestFrom(i2c_address,static_cast<uint8_t>(2)); //Retrieve 2 bytes from MPU6050

    if(Wire.available() >= 2){
        uint8_t first_byte = Wire.read();
        uint8_t second_byte = Wire.read();
        return ((first_byte << 8) | second_byte);
        // Converting retrieved value to Celcius        
    }
    return 0;
}

float mpu6050::getCalcTemp(){
    int16_t raw_temp_reading = this->getTemp();
    float convert_temp = (raw_temp_reading / 340.0) + 36.53;
    return convert_temp;
}

SensorData mpu6050::getAllSensorData(){
    SensorData data;
    Wire.beginTransmission(i2c_address);
    Wire.write(accel_x_h); // Start reading from the accel output register
    Wire.endTransmission(false);
    // Request sensor data (14 bytes in total)
    Wire.requestFrom(i2c_address, static_cast<uint8_t>(14));

    data.accel_x = Wire.read() << 8 | Wire.read();
    data.accel_y = Wire.read() << 8 | Wire.read();
    data.accel_z = Wire.read() << 8 | Wire.read();

    data.temperature = Wire.read() << 8 | Wire.read();

    data.gyro_x = Wire.read() << 8 | Wire.read();
    data.gyro_y = Wire.read() << 8 | Wire.read();
    data.gyro_z = Wire.read() << 8 | Wire.read();
    return data;
}

CalcSensorData mpu6050::getAllCalcSensorData(){
    CalcSensorData calc_data;
    // Get all raw data first
    SensorData raw_data = this->getAllSensorData();

    // Convert raw Accelerometer values to G-forces
    calc_data.calc_accel_x = ((float)raw_data.accel_x) / accel_lsb_to_g - accel_x_offset;
    calc_data.calc_accel_y = ((float)raw_data.accel_y) / accel_lsb_to_g - accel_y_offset;
    calc_data.calc_accel_z = (!upside_down_mount - upside_down_mount) * ((float)raw_data.accel_z) / accel_lsb_to_g - accel_z_offset;

    // Convert raw value to celcius
    calc_data.calc_temperature = (raw_data.temperature / 340.0) + 36.53;

    // Convert raw Gyroscope values to degrees per second
    calc_data.calc_gyro_x = ((float)raw_data.gyro_x) / gyro_lsb_to_degsec - gyro_x_offset;
    calc_data.calc_gyro_y = ((float)raw_data.gyro_y) / gyro_lsb_to_degsec - gyro_y_offset;
    calc_data.calc_gyro_z = ((float)raw_data.gyro_z) / gyro_lsb_to_degsec - gyro_z_offset;
    return calc_data;

}

uint8_t mpu6050::getGyroFullScaleRange(){
    uint8_t gyroConfig = 0;
    // Read the gyro configuration register to get the full-scale range setting
    Wire.beginTransmission(i2c_address);
    Wire.write(gyro_config);  // Address of the gyro configuration register
    Wire.endTransmission(false);
    Wire.requestFrom(i2c_address, static_cast<uint8_t>(1));  // Read one byte

    if (Wire.available()) {
        gyroConfig = Wire.read();
    }
    else{
        return 0;
    }
    // Extract the full-scale range setting from the gyro configuration
    return ((gyroConfig >> 3) & 0x03);
    }

uint8_t mpu6050::getSampleRate(){
    Wire.beginTransmission(i2c_address);
    Wire.write(sample_rate_div);
    // Send repeated start so communication doesn't stop before reading
    Wire.endTransmission(false);

    Wire.requestFrom(i2c_address,static_cast<uint8_t>(1)); //Retrieve 1 byte from MPU6050

    if(Wire.available()){
        uint8_t sample_rate_div_value = Wire.read();
        return sample_rate_div_value;
    }
    else{
        return 0;
    }
}

uint8_t mpu6050::getAccelFullScaleRange(){
    Wire.beginTransmission(i2c_address);
    Wire.write(accel_config);
    // Send repeated start so connection doesn't stop
    Wire.endTransmission(false);

    Wire.requestFrom(i2c_address,static_cast<uint8_t>(1)); // Read one byte from MPU6050

    if(Wire.available()){
        uint8_t value = Wire.read();
        // Extract the full_scale range from the register
        // And mask the remaining bits away
        uint8_t accel_full_scale_range = ((value >> 3) & 0x03);
        return accel_full_scale_range;
    }
    else{
        return 0;
    }
}

uint8_t mpu6050::getDLPF(){
    Wire.beginTransmission(i2c_address);
    Wire.write(dlpf_config);
    // Send repeated start so communication doesn't stop before reading
    Wire.endTransmission(false);
    Wire.requestFrom(i2c_address,static_cast<uint8_t>(1));

    if(Wire.available()){
        uint8_t dlpf_value = Wire.read();
        uint8_t mask = 0x07; // 00000111
        return (dlpf_value & mask);
    }
    else{
        return 0;
    }
}

void mpu6050::setClockSource(uint8_t clock_source_setting){
    if(clock_source_setting < 8 && clock_source_setting != 6){
        uint8_t current_power_setting = 0; //Default
        // Get current power management 1 setting
        Wire.beginTransmission(i2c_address);
        Wire.write(power_management1);
        Wire.endTransmission();
        if(Wire.available()){
            Wire.requestFrom(i2c_address,static_cast<uint8_t>(1));
            current_power_setting = Wire.read();
        }
        // Change only the clock source bits and leave the rest intact
        uint8_t mask = 0xF8; // 11111000
        current_power_setting &= mask;
        Wire.beginTransmission(i2c_address);
        Wire.write(power_management1);
        Wire.write(current_power_setting | clock_source_setting);
        Wire.endTransmission();
    }
    else{
        return; // Invalid parsed setting
    }
}

void mpu6050::setGyroFullScaleRange(uint8_t full_scale_range_setting){
    // Check if parsed argument is valid
    if(full_scale_range_setting <= 3){
        // Set gyro lsb to degrees per second to the correct new value
        if(full_scale_range_setting == 0){ // Range +-250 deg/s
            gyro_lsb_to_degsec = 131.0;
        }
        else if(full_scale_range_setting == 1){ // Range +-500 deg/s
            gyro_lsb_to_degsec = 65.5;
        }
        else if(full_scale_range_setting == 2){ // Range +-1000 deg/s
            gyro_lsb_to_degsec = 32.8;
        }
        else if(full_scale_range_setting == 3){ // Range +-2000 deg/s
            gyro_lsb_to_degsec = 16.4;
        }
        // Shift argument to the correct spot for the register
        full_scale_range_setting = (full_scale_range_setting << 3);
        Wire.beginTransmission(i2c_address);
        Wire.write(gyro_config);
        Wire.write(full_scale_range_setting);
        Wire.endTransmission();
    }
    else{
        // Input is invalid so don't change anything
        return;
    }
}

void mpu6050::setSampleRate(uint8_t sample_rate_setting){
    Wire.beginTransmission(i2c_address);
    Wire.write(sample_rate_div);
    Wire.write(sample_rate_setting);
    Wire.endTransmission();
}

void mpu6050::setAccelFullScaleRange(uint8_t full_scale_range_setting) {
    // Check if parsed argument is valid
    if(full_scale_range_setting <= 3){
        // Set accel lsb to g units to the correct new value
        if(full_scale_range_setting == 0){
            accel_lsb_to_g = 16384.0;
        }
        else if(full_scale_range_setting == 1){
            accel_lsb_to_g = 8192.0;
        }
        else if(full_scale_range_setting == 2){
            accel_lsb_to_g = 4096.0;
        }
        else if(full_scale_range_setting == 3){
            accel_lsb_to_g = 2048.0;
        }
        // Shift argument to the correct spot for the register
        full_scale_range_setting = (full_scale_range_setting << 3);
        Wire.beginTransmission(i2c_address);
        Wire.write(accel_config);
        Wire.write(full_scale_range_setting);
        Wire.endTransmission();
    }
    else{
        // Input is invalid, so don't change anything
        return;
    }
}

void mpu6050::setDLPF(uint8_t dlpf_setting){
    Wire.beginTransmission(i2c_address);
    Wire.write(dlpf_config);
    Wire.write(dlpf_setting);
    Wire.endTransmission();
}

void mpu6050::setFSYNC(uint8_t fsync_setting){
    Wire.beginTransmission(i2c_address);
    Wire.write(fsync_config);
    // Shift bits to the correct spot for the register
    fsync_setting = fsync_setting << 3;
    Wire.write(fsync_setting);
    Wire.endTransmission();
}