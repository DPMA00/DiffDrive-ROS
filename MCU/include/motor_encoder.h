#ifndef _MOTOR_ENCODER_H_
#define _MOTOR_ENCODER_H_

#include <Arduino.h>
#include <string>

struct FilterParams{
    float x_prev1;
    float x_prev2;
    float y_prev1;
    float y_prev2;
};

class MotorEncoder
{
private:
    int prev_encoder {0};
    std::string name;
    byte regStart;
    static constexpr int N {3};
    float b[N] = {0.06745527, 0.13491055, 0.06745527};
    float a[N] = {1.0, -1.1429805, 0.4128016};
    FilterParams filterparams;
    static constexpr int TICKS_PER_REV {360};
    static constexpr int ZEROVEL {128};

public:

    void set_prev_encoder(int val);
    int get_val();

    long readEncoder(byte MD25_ADDRESS);
    float computeRPM(long encoder_val, float dt);
    float filter(float x);

    MotorEncoder(std::string name, byte register_, FilterParams filterparams);
};

#endif
