#ifndef _MD25_H_
#define _MD25_H_

#include <Arduino.h>
#include "motor_controller.h"

struct Phi {
  float Phi1;
  float Phi2;
};


class MD25
{
private:
    byte speedregister {0x00};
    byte board_address {0x58};
    int ZEROVEL {128};
    const float base_L {0.29};
    const float wheel_R {0.05};
    const float max_cell_voltage {4.2};
    const float min_cell_voltage {3.2};

    float f_rpm1;
    float f_rpm2;
    float noisy_rpm1;
    float noisy_rpm2;
    float m1_current;
    float m2_current;
    float battery_voltage;
    float battery_power;
    MotorController &motor1;
    MotorController &motor2;

public:
    unsigned long prev_encoder_time;
    unsigned long prev_control_time;

    bool setup_drive(byte mode);
    Phi computePhi(float target_vx, float target_omega);
    Phi RadS2RPM(Phi Phi_rads);
    void run_control(float target_vx, float target_omega, float dt, const unsigned long now);
    bool run_encoders(bool first_run, const unsigned long now);
    void reset_encoders();
    float readMotorInfo(byte address);
    float calcBatteryPower(float battery_voltage);

    int get_encoder1();
    int get_encoder2();
    float get_rpm1();
    float get_rpm2();
    float get_current1();
    float get_current2();
    float get_battery_voltage();
    float get_battery_power();

    MD25(MotorController &m1, MotorController &m2);
};

#endif