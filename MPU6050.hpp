#ifndef MPU6050_HPP
#define MPU6050_HPP

#include <Arduino.h>
#include <Wire.h>

struct SensorData {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

struct CalcSensorData {
    float calc_accel_x;
    float calc_accel_y;
    float calc_accel_z;
    float calc_temperature;
    float calc_gyro_x;
    float calc_gyro_y;
    float calc_gyro_z;
};

class mpu6050{
private:
    uint8_t i2c_address;

    uint8_t power_management1 = 0x6B;
    uint8_t who_am_i = 0x75;

    uint8_t gyro_config = 0x1B;
    uint8_t accel_config = 0x1C;
    uint8_t sample_rate_div = 0x19;
    uint8_t dlpf_config = 0x1A;
    uint8_t fsync_config = 0X1;

    uint8_t accel_x_h = 0x3B;
    uint8_t accel_x_l = 0x3C;
    uint8_t accel_y_h = 0x3D;
    uint8_t accel_y_l = 0x3E;
    uint8_t accel_z_h = 0x3F;
    uint8_t accel_z_l = 0x40;

    uint8_t gyro_x_h = 0x43;
    uint8_t gyro_x_l = 0x44;
    uint8_t gyro_y_h = 0x45;
    uint8_t gyro_y_l = 0x46;
    uint8_t gyro_z_h = 0x47;
    uint8_t gyro_z_l = 0x48;

    float accel_x_offset = 0;
    float accel_y_offset = 0;
    float accel_z_offset = 0;

    float gyro_x_offset = 0;
    float gyro_y_offset = 0;
    float gyro_z_offset = 0;


    uint8_t temperature = 0x41;

    uint16_t calib_offset_amount = 500;
    float accel_lsb_to_g = 16384.0; //Default value with accel setting 0
    float gyro_lsb_to_degsec = 131.0; //Default value with gyro setting 0

    bool upside_down_mount = false;
public:
/**
 * @brief Default constructor
 * 
 * @param set_i2c_address 
 */
    mpu6050(uint8_t set_i2c_address = 0x68);

/**
 * @brief Wakes the device up by writing to the appropiate power management bit
 * 
 * @return 0 = Succes
 * 1 = Data too long
 * 2 = Received NACK for adress, I2C device did not acknowledge the adress indicating failure to establish communication
 * 3 = Received NACK for data, the I2C device did not acknowledge the data byte, indicating failure during data transmission
 */
    byte wakeUp();

/**
 * @brief Checks to see if the module is awake by trying to read the power management bit
 * 
 * @return true 
 * @return false 
 */
    bool awakeCheck();

/**
 * @brief This is a function that can ocasionally be called to check if communication with the module is still intact
 * 
 * @return 0 = Succes
 * 1 = Data too long
 * 2 = Received NACK for adress, I2C device did not acknowledge the adress indicating failure to establish communication
 * 3 = Received NACK for data, the I2C device did not acknowledge the data byte, indicating failure during data transmission 
 */
    byte communicationCheck();

/**
 * @brief Retrieve the module's I2C adress
 * 
 * @return an uint8_t number indicating the I2C adress of the module. 
 */
    uint8_t whoAmI();

/**
 * @brief Reset all internal registers back to their default state
 * 
 */
    void reset();

/**
 * @brief Calculate the offset of the module by taking a certain amount of measurements and taking the average
 * 
 * @param is_calc_acc Whether or not to calculate accelerometer offset
 * @param is_calc_gyro Whether or not to calculate gyroscope offset
 */
    void calcOffsets(bool is_calc_acc=true, bool is_calc_gyro=true);

/**
 * @brief Enable the accelerometer self test configuration
 * 
 */
    void enableAccelSelfTest();

/**
 * @brief Enable the gyroscope self test configuration
 * 
 */
    void enableGyroSelfTest();

/**
 * @brief Get the clock source setting from the power register
 * 
 * @return an uint8_t value indicating the current clock source setting
 * 0 = Internal 8Mhz oscillator
 * 1 = PLL with X axis gyroscope reference
 * 2 = PLL with Y axis gyroscope reference
 * 3 = PLL with Z axis gyroscope reference
 * 4 = PLL with external 32.768kHz reference
 * 5 = PLL with external 19.2MHz reference
 * 6 = Reserved
 * 7 = Stops the clock and keeps the timing generator in reset
 */
    uint8_t getClockSource();

/**
 * @brief Retrieve self test values from the registers and store them in the parsed parameters
 * 
 * @param accel_x 
 * @param accel_y 
 * @param accel_z 
 * @param gyro_x 
 * @param gyro_y 
 * @param gyro_z 
 */
    void getSelfTestValues(uint8_t & accel_x, uint8_t & accel_y, uint8_t & accel_z, uint8_t & gyro_x, uint8_t & gyro_y, uint8_t & gyro_z);

/**
 * @brief Get the raw Accelerometer X value
 * 
 * @return int16_t 
 */
    int16_t getAccelX();

/**
 * @brief Get the to g-forces converted X axis Accelerometer value
 * 
 * @return float 
 */
    float getCalcAccelX();

/**
 * @brief Get the raw Accelerometer Y value
 * 
 * @return int16_t 
 */
    int16_t getAccelY();

/**
 * @brief Get the to g-forces converted Y axis Accelerometer value
 * 
 * @return float 
 */
    float getCalcAccelY();

/**
 * @brief Get the raw Accelerometer Z value
 * 
 * @return int16_t 
 */
    int16_t getAccelZ();

/**
 * @brief Get the to g-forces converted Z axis Accelerometer value
 * 
 * @return float 
 */
    float getCalcAccelZ();

/**
 * @brief Get the raw Gyroscope X value
 * 
 * @return int16_t 
 */
    int16_t getGyroX();

/**
 * @brief Get the to degrees per second converted X axis Gyroscope value
 * 
 * @return float 
 */
    float getCalcGyroX();

/**
 * @brief Get the raw Gyroscope Y value
 * 
 * @return int16_t 
 */
    int16_t getGyroY();

/**
 * @brief Get the to degrees per second converted Y axis Gyroscope value
 * 
 * @return float 
 */
    float getCalcGyroY();

/**
 * @brief Get the raw Gyroscope Z value
 * 
 * @return int16_t 
 */
    int16_t getGyroZ();

/**
 * @brief Get the to degrees per second converted Z axis Gyroscope value
 * 
 * @return float 
 */
    float getCalcGyroZ();

/**
 * @brief Get the raw temperature reading
 * 
 * @return int_16t sensor temperature reading
 */
    int16_t getTemp();

/**
 * @brief Get the temperature converted to celcius
 * 
 * @return float converted temperature reading in celcius
 */
    float getCalcTemp();


/**
 * @brief Retrieve all sensor (accel, gyro, temp) data in one go
 * 
 * @return A struct storing all the sensor reading values
 */
    SensorData getAllSensorData();

/**
 * @brief Retrieve and convert all sensor (accel, gyro, temp) data in one go
 * 
 * @return A struct storing all the converted sensor reading values (G-force, Celcius, Degrees per second and celcius)
 */
    CalcSensorData getAllCalcSensorData();

/**
 * @brief Get the Accel Full Scale Range value
 * 
 * @return an uint8_t value indicating the sensitivity or scale factor of the gyroscope sensor.
 * The following values correspond to the following settings:
 * 0 = ±2g (default).
 * 1 = ±4g.
 * 2 = ±8g.
 * 3 = ±16g.
 */
    uint8_t getAccelFullScaleRange();

/**
 * @brief Get the Gyro full-scale range value 
 * 
 * @return an uint8_t value indicating the sensitivity or scale factor of the gyroscope sensor.
 * The following values correspond to the following settings:
 * 0 = full-scale range of ±250°/s, the sensitivity is 131 LSB per °/s (default).
 * 1 = full-scale range of ±500°/s, the sensitivity is 65.5 LSB per °/s.
 * 2 = full-scale range of ±1000°/s, the sensitivity is 32.8 LSB per °/s.
 * 3 = full-scale range of ±2000°/s, the sensitivity is 16.4 LSB per °/s.
 */
    uint8_t getGyroFullScaleRange();

/**
 * @brief Get the Gyroscope sampling rate
 * 
 * @return an uint8_t representing the sampling rate for the Gyro sensor.
 * Sample rate = Gyroscope output rate / (1+ SMPLRT_DIV)
 * The following values correspond to the following settings:
 * 8kHz sample rate: SMPLRT_DIV = 0x00 (Default with DMP disabled) 
 * 1kHz sample rate: SMPLRT_DIV = 0x07
 * 500Hz sample rate: SMPLRT_DIV = 0x0F
 * 200Hz sample rate: SMPLRT_DIV = 0x27
 * 100Hz sample rate: SMPLRT_DIV = 0x4F
 * 50Hz sample rate: SMPLRT_DIV = 0x9F
 */
    uint8_t getSampleRate();

/**
 * @brief Get the Digital Low Pass Filter value
 * 
 * @return an uint8_t value indicating the current DLPF setting.
 * By default DLPF is disabled (0x00) with no filtering.
 * The values correspond to the following settings:
 *           - Accelerometer -
 * 0x00: Bandwidth = 260Hz, Delay = 0ms
 * 0x01: Bandwidth = 184Hz, Delay = 2.0ms
 * 0x02: Bandwidth = 94Hz, Delay = 3.0ms
 * 0x03: Bandwidth = 44Hz, Delay = 4.9ms
 * 0x04: Bandwidth = 21Hz, Delay = 8.5ms
 * 0x05: Bandwidth = 10Hz, Delay = 13.8ms
 * 0x06: Bandwidth = 5Hz, Delay = 19.0ms
 * 0x07: Reserved
 *            - Gyroscope -
 * 0x00: Bandwidth = 256Hz, Delay = 0.98ms
 * 0x01: Bandwidth = 188Hz, Delay = 1.9ms
 * 0x02: Bandwidth = 98Hz, Delay = 2.8ms
 * 0x03: Bandwidth = 42Hz, Delay = 4.8ms
 * 0x04: Bandwidth = 20Hz, Delay = 8.3ms
 * 0x05: Bandwidth = 10Hz, Delay = 13.4ms
 * 0x06: Bandwidth = 5Hz, Delay = 18.6ms
 * 0x07: Reserved
 */
    uint8_t getDLPF();

/**
 * @brief Set the Clock Source setting
 * 
 * @param an uint8_t value indicating which setting you want
 * 0 = Internal 8Mhz oscillator
 * 1 = PLL with X axis gyroscope reference
 * 2 = PLL with Y axis gyroscope reference
 * 3 = PLL with Z axis gyroscope reference
 * 4 = PLL with external 32.768kHz reference
 * 5 = PLL with external 19.2MHz reference
 * 6 = Reserved
 * 7 = Stops the clock and keeps the timing generator in reset
 */
    void setClockSource(uint8_t clock_source_setting);

/**
 * @brief Set the Accelerometer Full Scale Range configuration
 * 
 * @param Parse the desired full-scale range setting.
  * The following values correspond to the following settings:
 * 0 = ±2g (default).
 * 1 = ±4g.
 * 2 = ±8g.
 * 3 = ±16g.
 */
    void setAccelFullScaleRange(uint8_t full_scale_range_setting);

/**
 * @brief Set the Gyroscope Full Scale Range configuration
 * 
 * @param Parse the desired full-scale range setting.
 * The following values correspond to the following settings:
 * 0 = full-scale range of ±250°/s, the sensitivity is 131 LSB per °/s (default).
 * 1 = full-scale range of ±500°/s, the sensitivity is 65.5 LSB per °/s.
 * 2 = full-scale range of ±1000°/s, the sensitivity is 32.8 LSB per °/s.
 * 3 = full-scale range of ±2000°/s, the sensitivity is 16.4 LSB per °/s.
 */
    void setGyroFullScaleRange(uint8_t full_scale_range_setting);

/**
 * @brief Set the Gyroscope Sample Rate configuration
 * 
 * @param Parse the desired full-scale range setting in hexadecimal value.
 * For example for setting 1kHZ: 0x07
 */
    void setSampleRate(uint8_t sample_rate_setting);

/**
 * @brief Set the Digital Low Pass Filter setting
 * 
 * @param Parse the desired digital low pass filter setting in hexadecimal value.
 * For example for setting 3 parse: 0x03. 
 */
    void setDLPF(uint8_t dlpf_setting);
    
/**
 * @brief Set the Frame Synchronization setting
 * 
 * @param Parse the desired synchronization setting in hexadecimal value.
 * For example for setting 6 parse: 0x06. 
 */
    void setFSYNC(uint8_t fsync_setting);

    float getAccelXOffset(){ return accel_x_offset;};
    float getAccelYOffset(){ return accel_y_offset;};
    float getAccelZOffset(){ return accel_z_offset;};

    float getGyroXOffset(){ return gyro_x_offset;};
    float getGyroYOffset(){ return gyro_y_offset;};
    float getGyroZOffset(){ return gyro_z_offset;};
};

#endif