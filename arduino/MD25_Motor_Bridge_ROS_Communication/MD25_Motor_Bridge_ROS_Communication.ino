#include <Wire.h>
#include <stdint.h>

#define MD25_ADDRESS 0x58
#define TICKS_PER_REV 360
#define ZEROVEL 128
#define ENCODER_INTERVAL 10 //ms
#define CONTROL_INTERVAL 20 //ms


const float base_L = 0.29;
const float wheel_R = 0.05;

/* Encoder state */
unsigned long prev_encoder_time = 0;
unsigned long prev_control_time = 0;
int32_t prev_encoder1 = 0;
int32_t prev_encoder2 = 0;

/* Velocities */

float rpm1 = 0;
float rpm2 = 0;

volatile float target_vx =0;// = 0.1; // max velocity about 1ms-1
volatile float target_omega =0;// = 0; // max angular velocty about 13.3 rads-1


/* PID Parameters */

float Kp = 0.2;
float Ki = 1.2;
float Kd = 1.0;

float prev_error1 = 0;
float prev_error2 = 0;
float error_integral1 = 0;
float error_integral2 = 0;

float max_integral = 500.0;

float smoothed_rpm1 = 0;
float smoothed_rpm2 = 0;
const float alpha = 0.2;  // smoothing factor


/* Helper functions */

struct Phi {
  float Phi1;
  float Phi2;
};


void setup() {
  Wire.begin();
  Serial.begin(57600);
  Serial.println("===== Arduino booted =====");
  delay(500);
  // Set mode 0 (Motor1 = reg 0, Motor2 = reg 1)
  Wire.beginTransmission(MD25_ADDRESS);
  Wire.write(0x0F); // Mode register
  Wire.write(0x00); // Mode 0
  Wire.endTransmission();

  Wire.beginTransmission(MD25_ADDRESS);
  Wire.write(0x10); // Command register
  Wire.write(0x20); // Reset encoders
  Wire.endTransmission();

  // Initialize motors to standstill
  Wire.beginTransmission(MD25_ADDRESS);
  Wire.write(0x00);         // Speed1 register
  Wire.write(ZEROVEL);      // Motor 1
  Wire.write(ZEROVEL);      // Motor 2
  Wire.endTransmission();

  prev_encoder_time = millis();
  prev_control_time = millis();
}

long readEncoder(byte regStart) {
  Wire.beginTransmission(MD25_ADDRESS);
  Wire.write(regStart);
  Wire.endTransmission();

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

int computePID(float target_rpm, float actual_rpm, float &integral_error, float &prev_error, float dt){
  float error = target_rpm - actual_rpm;
  integral_error += error * dt;
  integral_error = constrain(integral_error, -max_integral, max_integral);

  float d_error = error - prev_error;

  prev_error = error;
  float control = Kp * error + Ki * integral_error + Kd * d_error;
  control = constrain(control, -127, 127);
  if (abs(control) <5)
  {
    control = 0;
  } 
  return (int)control;
}

float computeRPM(long encoder_val, long prev_encoder, float dt) {
  if (dt <= 0) return 0;

  long d_encoder = encoder_val - prev_encoder;

  if (abs(d_encoder) > 1000) {
    //Serial.println("Spike detected in encoder!");
    return 0;
  }

  return (d_encoder / (float)TICKS_PER_REV) * (60.0 / dt);
}


Phi computePhi(float target_vx, float target_omega)
{
  Phi values;

  values.Phi1 = target_vx/wheel_R - base_L/2 * target_omega/wheel_R;
  values.Phi2 = target_vx/wheel_R + base_L/2 * target_omega/wheel_R;

  return values;
  
}


Phi RadS2RPM(Phi Phi_rads)
{
  Phi_rads.Phi1 = Phi_rads.Phi1 * 60 / (2*PI);
  Phi_rads.Phi2 = Phi_rads.Phi2 * 60 / (2*PI); 

  return Phi_rads;
}


void loop() {

  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    
    line.trim();  // clean up extra spaces or newlines

    if (line == "r") {
    // Reset encoder counts
    prev_encoder1 = 0;
    prev_encoder2 = 0;

    // Optional: reset filtered variables too
    smoothed_rpm1 = 0;
    smoothed_rpm2 = 0;
    error_integral1 = 0;
    error_integral2 = 0;
    return;
    }
    
    int spaceIndex = line.indexOf(' ');
    if (spaceIndex != -1) {
      String vxStr = line.substring(0, spaceIndex);
      String omegaStr = line.substring(spaceIndex + 1);
    
      target_vx = -vxStr.toFloat();
      target_omega = -omegaStr.toFloat();
    } else {
      Serial.println("Malformed input. Could not parse target velocities.");
    }


  }
    
  static bool first_run = true;
  
  unsigned long now = millis();
  
  if (now - prev_encoder_time >= ENCODER_INTERVAL) {
      long enc1 = readEncoder(0x02);
      long enc2 = readEncoder(0x06);
  
      float dt_encoder = (now - prev_encoder_time) / 1000.0;
  
      if (enc1 != -1 && enc2 != -1) {
  
        if (first_run)
        {
          prev_encoder1 = enc1;
          prev_encoder2 = enc2;
  
          smoothed_rpm1 = 0;
          smoothed_rpm2 = 0;
  
          first_run = false;
        }
        
        rpm1 = computeRPM(enc1, prev_encoder1, dt_encoder);
        rpm2 = computeRPM(enc2, prev_encoder2, dt_encoder);
  
        smoothed_rpm1 = alpha * rpm1 + (1 - alpha) * smoothed_rpm1;
        smoothed_rpm2 = alpha * rpm2 + (1 - alpha) * smoothed_rpm2;
  
        prev_encoder1 = enc1;
        prev_encoder2 = enc2;
  
        prev_encoder_time = now;
      }
    }

  if (now - prev_control_time >= CONTROL_INTERVAL) {
    float dt_control = (now - prev_control_time) / 1000.0;

    Phi phi_rads = computePhi(target_vx, target_omega);
    Phi phi_rpm = RadS2RPM(phi_rads);
    
    int motor1_out = computePID(phi_rpm.Phi1, smoothed_rpm1, error_integral1, prev_error1, dt_control);
    int motor2_out = computePID(phi_rpm.Phi2, smoothed_rpm2, error_integral2, prev_error2, dt_control);

    Wire.beginTransmission(MD25_ADDRESS);
    Wire.write(0x00);  // Speed register
    Wire.write(ZEROVEL + motor1_out);
    Wire.write(ZEROVEL + motor2_out);
    Wire.endTransmission();

    Serial.print('$');
    Serial.print(prev_encoder2);
    Serial.print(" ");
    Serial.print(prev_encoder1);
    Serial.print(" ");
    Serial.print(smoothed_rpm2);
    Serial.print(" ");
    Serial.print(smoothed_rpm1);
    Serial.print(" ");
    Serial.print(1.0); // Placeholder for left current
    Serial.print(" ");
    Serial.print(1.0); // Placeholder for right current
    Serial.print(" ");
    Serial.println(1.0); // Placeholder for battery

    prev_control_time = now;
  }
}
