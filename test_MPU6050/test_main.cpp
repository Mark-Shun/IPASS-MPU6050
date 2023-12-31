#include <unity.h>
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.hpp"

// Test MPU6050's implementation using the Unity unit testing framework

mpu6050 testSensor(0x68);

void setUp(void)
{
  // Execute code before every test
  testSensor.wakeUp();
}

void tearDown(void)
{
  // clean stuff up here
}

void testWakeUp() {

  // Test the wakeUp() function
  bool check = testSensor.awakeCheck();
  TEST_ASSERT_TRUE(check);
}

void testWhoAmI() {
  // Test the whoAmI() function
  uint8_t result = testSensor.whoAmI();
  TEST_ASSERT_EQUAL_UINT8(0x68,result);
}

void testDLPF(){
  testSensor.setDLPF(0x05);
  uint8_t result = testSensor.getDLPF();
  TEST_ASSERT_EQUAL_UINT8(0x05,result);
}

void testGyroFullScaleRange(){
  testSensor.setGyroFullScaleRange(0x03);
  uint8_t result = testSensor.getGyroFullScaleRange();
  TEST_ASSERT_EQUAL_UINT8(0x03,result);
}

void testAccelFullScaleRange(){
  testSensor.setAccelFullScaleRange(0x02);
  uint8_t result = testSensor.getAccelFullScaleRange();
  TEST_ASSERT_EQUAL_UINT8(0x02,result);
}

void testSampleRate(){
  testSensor.setSampleRate(0x01);
  uint8_t result = testSensor.getSampleRate();
  TEST_ASSERT_EQUAL_UINT8(0x01,result);
}

void testModuleSelfTest(){
  const float tolerance = 0.14;

  uint8_t testAccelX = 0;
  uint8_t testAccelY = 0;
  uint8_t testAccelZ = 0;

  float trimAccelX = 0.0;
  float trimAccelY = 0.0;
  float trimAccelZ = 0.0;

  uint8_t testGyroX = 0;
  uint8_t testGyroY = 0;
  uint8_t testGyroZ = 0;

  float trimGyroX = 0.0;
  float trimGyroY = 0.0;
  float trimGyroZ = 0.0;

  testSensor.enableAccelSelfTest();
  testSensor.enableGyroSelfTest();

  delay(100);

  testSensor.getSelfTestValues(testAccelX,testAccelY,testAccelZ,testGyroX,testGyroY,testGyroZ);

  uint8_t testValues[] = {testAccelX,testAccelY,testAccelZ,testGyroX,testGyroY,testGyroZ};
  float trimValues[] = {trimAccelX,trimAccelY,trimAccelZ,trimGyroX,trimGyroY,trimGyroZ};
  float percentages[6];
  
  // From test value to factory trim calculation
  for(unsigned int i = 0; i < 6; i++){
    // Factory trim Acceleration calculation
    if(i<3){
      trimValues[i] = (4096.0 * 0.34) * pow((0.92 / 0.34), (((float)testValues[i]) - 1.0) / 30.0);
      // trimValues[i] = (4096.0 * 0.34) * ( pow((0.92 / 0.34), ((float)testValues[i] - 1.0 / 30.0)));
    }
    // Factory trim Gyro calculation
    else if(i>=3 && i!=4){
      trimValues[i] = ( 25.0 * 131.0) * (pow( 1.046 , (((float)testValues[i]) - 1.0) ));
    }
    // Factory trim Gyro Y has a different formula
    else{
      trimValues[i] = (-25.0 * 131.0) * (pow( 1.046 , (((float)testValues[i]) - 1.0) ));
    }
  }

  // Percentages calculations
  for(unsigned int i = 0; i < 6; i++){
    percentages[i] = 100.0 + 100.0 * (((float)testValues[i]) - trimValues[i]) / trimValues[i];
  }

  // Print values to check them in serial monitor
  Serial.println("-Test values-");
  for(unsigned int i =0;i<6;i++){
    Serial.print(i); Serial.print(": "); Serial.println(testValues[i]);
  }

  Serial.println("-Trim values-");
  for(unsigned int i =0;i<6;i++){
    Serial.print(i); Serial.print(": "); Serial.println(trimValues[i]);
  }
  
  Serial.println("-Percentages-");
  for(unsigned int i =0;i<6;i++){
    Serial.print(i); Serial.print(": "); Serial.println(percentages[i]);
  }

  // Perform floating-point comparisons
  TEST_ASSERT_FLOAT_WITHIN(tolerance, 0, percentages[0]);
  TEST_ASSERT_FLOAT_WITHIN(tolerance, 0, percentages[1]);
  TEST_ASSERT_FLOAT_WITHIN(tolerance, 0, percentages[2]);
  TEST_ASSERT_FLOAT_WITHIN(tolerance, 0, percentages[3]);
  TEST_ASSERT_FLOAT_WITHIN(tolerance, 0, percentages[4]);
  TEST_ASSERT_FLOAT_WITHIN(tolerance, 0, percentages[5]);

}

// Define the necessary Arduino functions
void setup() {
    Serial.begin(115200);
    delay(500);
    UNITY_BEGIN();
}

void loop() {
    // Run the tests
    RUN_TEST(testWakeUp);
    RUN_TEST(testWhoAmI);
    RUN_TEST(testDLPF);
    RUN_TEST(testGyroFullScaleRange);
    RUN_TEST(testAccelFullScaleRange);
    RUN_TEST(testSampleRate);
    RUN_TEST(testModuleSelfTest);
  
    UNITY_END();
}
