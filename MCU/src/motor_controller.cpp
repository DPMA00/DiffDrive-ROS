#include "motor_controller.h"

MotorController::MotorController(std::string name, MotorEncoder &encoder)
: name{name}, encoder{encoder}
{
}

void MotorController::set_PID_params(float Kp, float Ki, float Kd, float max_integral)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->max_integral = max_integral;

}



int MotorController::computePID(float target_rpm, float actual_rpm, float dt)
{
    float error = target_rpm - actual_rpm;
    integral_error += error * dt;
    integral_error = constrain(integral_error, -max_integral, max_integral);

    float d_error = error - prev_error;

    prev_error = error;
    float control = Kp * error + Ki * integral_error + Kd * d_error;
    control = constrain(control, -110, 110);

    if (abs(control) <5) control = 0;
    return (int)control;
}