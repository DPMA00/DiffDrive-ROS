#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

struct IMU_data {
    float acc_x,acc_y,acc_z;
    float omega_x, omega_y, omega_z;
    float temp;
};


class IMU 
{
private:
    Adafruit_MPU6050 mpu;
    IMU_data data;
    
public:
    void setup();
    unsigned long prev_imu_time {0};
    IMU_data get_reading(unsigned long now);
    IMU(Adafruit_MPU6050 &mpu);
};

#endif