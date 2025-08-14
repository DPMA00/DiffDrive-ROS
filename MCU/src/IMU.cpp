#include "IMU.h"

IMU::IMU(Adafruit_MPU6050 &mpu)
: mpu(mpu)
{

}

void IMU::setup()
{
    if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
    }
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
}

IMU_data IMU::get_reading(unsigned long now)
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    data.acc_x = a.acceleration.x;
    data.acc_y = a.acceleration.y;
    data.acc_z = a.acceleration.z;
    data.omega_x = g.gyro.x;
    data.omega_y = g.gyro.y;
    data.omega_z = g.gyro.z;
    data.temp = temp.temperature;

    prev_imu_time = now;
    return data;
}

