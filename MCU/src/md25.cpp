#include "md25.h"
#include <Wire.h>

MD25::MD25(MotorController &m1, MotorController &m2)
: motor1{m1}, motor2{m2}
{

}

bool MD25::setup_drive(byte mode)
{
    Wire.beginTransmission(board_address);
    Wire.write(0x0F); // Mode register
    Wire.write(mode); // Mode 0
    if (Wire.endTransmission() !=0){
        return false;
    };

    Wire.beginTransmission(board_address);
    Wire.write(0x00);         // Speed1 register
    Wire.write(ZEROVEL);      // Motor 1
    Wire.write(ZEROVEL);      // Motor 2
    if (Wire.endTransmission() != 0){
        return false;
    };

    motor1.encoder.set_prev_encoder(0);
    motor2.encoder.set_prev_encoder(0);

    prev_control_time = millis();
    prev_encoder_time = millis();

    return true;
}

// Inverse kinematics
Phi MD25::computePhi(float target_vx, float target_omega)
{
  Phi values;

  values.Phi1 = target_vx/wheel_R - base_L/2 * target_omega/wheel_R;
  values.Phi2 = target_vx/wheel_R + base_L/2 * target_omega/wheel_R;

  return values;
  
}

// Target speeds to RPM

Phi MD25::RadS2RPM(Phi Phi_rads)
{
  Phi_rads.Phi1 = Phi_rads.Phi1 * 60 / (2*PI);
  Phi_rads.Phi2 = Phi_rads.Phi2 * 60 / (2*PI); 

  return Phi_rads;
}

bool MD25::run_encoders(bool first_run, const unsigned long now)
{
  long enc1 = motor1.encoder.readEncoder(board_address);
  long enc2 = motor2.encoder.readEncoder(board_address);

  float dt_encoder = (now - prev_encoder_time) / 1000.0;

  if (enc1 != -1 && enc2 != -1) {

    if (first_run)
    {
      motor1.encoder.set_prev_encoder(enc1);
      motor2.encoder.set_prev_encoder(enc2);

      f_rpm1 = 0;
      f_rpm2 = 0;

      first_run = false;
    }
    
    noisy_rpm1 = motor1.encoder.computeRPM(enc1, dt_encoder);
    noisy_rpm2 = motor2.encoder.computeRPM(enc2, dt_encoder);
    
    f_rpm1 = motor1.encoder.filter(noisy_rpm1);
    f_rpm2 = motor2.encoder.filter(noisy_rpm2);
    
    motor1.encoder.set_prev_encoder(enc1);
    motor2.encoder.set_prev_encoder(enc2);

    m1_current = readMotorInfo(0x0B);
    m2_current = readMotorInfo(0x0C);
    battery_voltage = readMotorInfo(0x0A);
    battery_power = calcBatteryPower(battery_voltage);
  }
  prev_encoder_time = now;
  
  return first_run;
}


void MD25::run_control(float target_vx, float target_omega, float dt, const unsigned long now)
{
    Phi phi_rads = computePhi(target_vx, target_omega);
    Phi phi_rpm = RadS2RPM(phi_rads);
    
    int motor1_out = motor1.computePID(phi_rpm.Phi1, f_rpm1, dt);
    int motor2_out = motor2.computePID(phi_rpm.Phi2, f_rpm2, dt);

    Wire.beginTransmission(board_address);
    Wire.write(0x00);  // Speed register
    Wire.write(ZEROVEL + motor1_out); 
    Wire.write(ZEROVEL + motor2_out);
    Wire.endTransmission();
    prev_control_time = now;
}

void MD25::reset_encoders(){
  motor1.encoder.set_prev_encoder(0);
  motor2.encoder.set_prev_encoder(0);
  
  Wire.beginTransmission(board_address);
  Wire.write(0x10); 
  Wire.write(0x20);  
  Wire.endTransmission();
}

float MD25::readMotorInfo(byte address){
  Wire.beginTransmission(board_address);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(board_address, 1);
  if (Wire.available()){
    uint8_t val = Wire.read();
    return val/10;
  }
  return -1.0;
}

// Calculate 3S lipo battery percentage
float MD25::calcBatteryPower(float battery_voltage){

  float curr_volt_diff = battery_voltage - 3* min_cell_voltage;
  float max_volt_diff = 3*max_cell_voltage - 3*min_cell_voltage;
  
  return curr_volt_diff/max_volt_diff *100;
}

int MD25::get_encoder1()
{
  return motor1.encoder.get_val();
}

int MD25::get_encoder2()
{
  return motor2.encoder.get_val();
}

float MD25::get_rpm1()
{
  return noisy_rpm1;
}

float MD25::get_rpm2()
{
  return noisy_rpm2;
}

float MD25::get_current1()
{
  return m1_current;
}
    
float MD25::get_current2()
{
  return m2_current;
}

float MD25::get_battery_voltage()
{
  return battery_voltage;
}

float MD25::get_battery_power()
{
  return battery_power;
}