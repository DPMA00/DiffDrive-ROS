#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <Arduino.h>
#include <string>
#include "motor_controller.h"
#include "motor_encoder.h"

class MotorController
{
private:
    float Kp;
    float Ki;
    float Kd;

    float integral_error {0};
    float prev_error {0};
    float max_integral;


public:
    std::string name;
    MotorEncoder &encoder;

    int computePID(float target_rpm, float actual_rpm, float dt);
    void set_PID_params(float Kp, float Ki, float Kd, float max_integral);

MotorController(std::string name, MotorEncoder &encoder);
};

#endif