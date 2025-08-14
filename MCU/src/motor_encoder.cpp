#include "motor_encoder.h"
#include <Wire.h>

MotorEncoder::MotorEncoder(std::string name, byte register_, FilterParams filterparams)
: name{name}, regStart(register_), filterparams(filterparams){}

void MotorEncoder::set_prev_encoder(int val)
{
    prev_encoder = val;
}

int MotorEncoder::get_val()
{
    return prev_encoder;
}


// Encoder readings
long MotorEncoder::readEncoder(byte MD25_ADDRESS)
{
    Wire.beginTransmission(MD25_ADDRESS);
    Wire.write(regStart);
    Wire.endTransmission(false);
    Wire.requestFrom(MD25_ADDRESS, 4);
    if (Wire.available() < 4) {
    Serial.println("I2C read failed!");
    return -1;
    }

    long val = 0;
    for (int i = 0; i < 4; i++) {
    val <<= 8;
    val |= Wire.read();
    }
    return val;
}

// RPM Calculation
float MotorEncoder::computeRPM(long encoder_val, float dt) {
  if (dt <= 0) return 0;

  long d_encoder = encoder_val - prev_encoder;

  if (abs(d_encoder) > 100000) {
    //Serial.println("Spike detected in encoder!");
    return 0;
  }

  return (d_encoder / (float)TICKS_PER_REV) * (60.0 / dt);
}



// Velocity smoothing filter
float MotorEncoder::filter(float x)
{
  float y = b[0]*x + b[1]*filterparams.x_prev1 + b[2]*filterparams.x_prev2 - a[1]*filterparams.y_prev1 - a[2]*filterparams.y_prev2;

  filterparams.x_prev2 = filterparams.x_prev1;
  filterparams.x_prev1 = x;
  filterparams.y_prev2 = filterparams.y_prev1;
  filterparams.y_prev1 = y;

  return y;
  
}