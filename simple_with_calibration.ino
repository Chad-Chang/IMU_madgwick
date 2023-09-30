#include "MPU9250_1.h"
#include <IMU.h>

MPU9250 mpu;
cIMU    IMU;
uint8_t   err_code;
uint8_t   led_tog = 0;
uint8_t   led_pin = 13;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    IMU.begin();
    pinMode( led_pin, OUTPUT );

    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    print_calibration();
    mpu.verbose(false);
}

void loop() {
  static uint32_t tTime[3];
  static uint32_t imu_time = 0;
  
  if( (millis()-tTime[0]) >= 500 )
  {
    tTime[0] = millis();

    digitalWrite( led_pin, led_tog );
    led_tog ^= 1;
  }

  tTime[2] = micros();
  if( IMU.update() > 0 ) imu_time = micros()-tTime[2];




  if( (millis()-tTime[1]) >= 50 )
  {
    tTime[1] = millis();

    // Serial.print(imu_time);
    // Serial.print(" ");
    // Serial.print(IMU.rpy[0]);
    // Serial.print(" ");
    // Serial.print(IMU.rpy[1]);
    // Serial.print(" ");
    // Serial.println(IMU.rpy[2]);
  }


  if( Serial.available() )
  {
    char Ch = Serial.read();

    if( Ch == '1' )
    {
      Serial.println("ACC Cali Start");

      IMU.SEN.acc_cali_start();
      while( IMU.SEN.acc_cali_get_done() == false )
      {
        IMU.update();
      }

      Serial.print("ACC Cali End ");
    }
  }


    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_roll_pitch();
            prev_ms = millis();
        }
    }
}

void print_roll_pitch() {
    Serial.print("Pitch, Roll: ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    
}
