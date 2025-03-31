#include <Wire.h>
#define MD25_ADDRESS 0x58
#define TICKS_PER_REV 360
#define ZEROVEL 128
#define ENCODER_INTERVAL 10
#define CONTROL_INTERVAL 20

unsigned long prev_encoder_time = 0;
unsigned long prev_control_time = 0;
long prev_encoder1 = 0;
long prev_encoder2 = 0;
float rpm1 = 0;
float rpm2 = 0;

float target_rpm1 = 40;
float target_rpm2 = 80;

float Kp = 0.8;
float Ki = 0.3;
float Kd = 0.1;

float prev_error1 = 0;
float prev_error2 = 0;
float error_integral1 = 0;
float error_integral2 = 0;

float max_integral = 500.0;

float smoothed_rpm1 = 0;
float smoothed_rpm2 = 0;
float alpha = 0.25;  // smoothing factor

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(500);

  Serial.println("Setting MD25 to mode 0...");

  // Set mode 0 (Motor1 = reg 0, Motor2 = reg 1)
  Wire.beginTransmission(MD25_ADDRESS);
  Wire.write(0x0F); // Mode register
  Wire.write(0x00); // Mode 0
  Wire.endTransmission();

  // Initialize motors to standstill
  Wire.beginTransmission(MD25_ADDRESS);
  Wire.write(0x00);         // Speed1 register
  Wire.write(ZEROVEL);      // Motor 1
  Wire.write(ZEROVEL);      // Motor 2
  Wire.endTransmission();

  prev_encoder_time = millis();
  prev_control_time = millis();
  Serial.println("Motors initialized. Reading encoders...");
}

long readEncoder(byte regStart) {
  Wire.beginTransmission(MD25_ADDRESS);
  Wire.write(regStart);
  Wire.endTransmission();

  Wire.requestFrom(MD25_ADDRESS, 4);
  if (Wire.available() < 4) {
    Serial.println("⚠️ I2C read failed!");
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
  return (int)control;
}

float computeRPM(long encoder_val, long prev_encoder, float dt) {
  if (dt <= 0) return 0;

  long d_encoder = encoder_val - prev_encoder;

  if (abs(d_encoder) > 100000) {
    Serial.println("Spike detected in encoder!");
    return 0;
  }

  return (d_encoder / (float)TICKS_PER_REV) * (60.0 / dt);
}

void loop() {
  unsigned long now = millis();
  
  if (now - prev_encoder_time >= ENCODER_INTERVAL) {
    long enc1 = readEncoder(0x02);
    long enc2 = readEncoder(0x06);

    float dt_encoder = (now - prev_encoder_time) / 1000.0;

    if (enc1 != -1 && enc2 != -1) {
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

    int motor1_out = computePID(target_rpm1, smoothed_rpm1, error_integral1, prev_error1, dt_control);
    int motor2_out = computePID(target_rpm2, smoothed_rpm2, error_integral2, prev_error2, dt_control);

    Wire.beginTransmission(MD25_ADDRESS);
    Wire.write(0x00);  // Speed register
    Wire.write(ZEROVEL + motor1_out);
    Wire.write(ZEROVEL + motor2_out);
    Wire.endTransmission();

    // 💥 Output for Serial Plotter (RPM1, RPM2, Targets)
    Serial.print(smoothed_rpm1);
    Serial.print("\t");
    Serial.print(smoothed_rpm2);
    Serial.print("\t");
    Serial.print(target_rpm1);
    Serial.print("\t");
    Serial.println(target_rpm2);

    prev_control_time = now;
  }
}
