#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include "md25.h"
#include "IMU.h"


//---- Motor speed control parameters -------//
const int SENSOR_INTERVAL = 10; //10
const int CONTROL_INTERVAL = 30; //30
const int IMU_INTERVAL = 30;
bool manual_mode = false;

const float base_L = 0.29;
const float wheel_R = 0.05;


float target_vx = 0.0; // max velocity about 0.9ms-1 (negate this because inverse positioning of motors)
float target_omega = 0.0; // max angular velocty about 13.3 rads-1

FilterParams f1 = {0.0f, 0.0f, 0.0f, 0.0f};
FilterParams f2 = {0.0f, 0.0f, 0.0f, 0.0f};

float Kp_1 = 0.4;
float Ki_1 = 2.2;
float Kd_1 = 0.5*Kp_1;

float Kp_2 = 0.4;
float Ki_2 = 2.2;
float Kd_2 = 0.5*Kp_2;

float max_integral = 50.0;

IMU_data cachedIMU;

Adafruit_MPU6050 mpu;


//-------ESP-ESP Communication----------//
typedef struct struct_msg{
  float vx;
  float omega;
  bool z;
} struct_msg;

struct_msg myData;
bool last_z_state = false;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void sendStatus();

MotorEncoder encoder1("encoder1",0x02, f1);
MotorEncoder encoder2("encoder2",0x06, f2);
MotorController motor1("motor1", encoder1);
MotorController motor2("motor2", encoder2);
MD25 motor_board(motor1,motor2);
IMU imu(mpu);

//---------SETUP---------//

void setup() {
  delay(100);

  Serial.begin(115200);
  Wire.begin(4, 5);
  Wire.setClock(100000);
  delay(100);
  
  imu.setup();

  WiFi.mode(WIFI_STA);

  if (motor_board.setup_drive(0x00) !=1){
    Serial.println("Error initializing motor board");
    return;
  }
  motor_board.reset_encoders();
  Serial.println("Motors initialized. Reading encoders...");
  motor1.set_PID_params(Kp_1,Ki_1,Kd_1,max_integral);
  motor2.set_PID_params(Kp_2,Ki_2,Kd_2,max_integral);

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}



// ------------- MAIN LOOP ----------------//
void loop() {
  
  if (manual_mode)
  {
    target_vx = -myData.vx; //negate as wheels are mounted inversely
    target_omega = myData.omega;
  }
  
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();  // clean up extra spaces or newlines

    //check encoder reset command
    if (line.equalsIgnoreCase("r") || line.equalsIgnoreCase("reset")){
      motor_board.reset_encoders();
      return;
    }

    // velocity
    int spaceIndex = line.indexOf(' ');
    if (spaceIndex != -1) {
      String vxStr = line.substring(0, spaceIndex);
      String omegaStr = line.substring(spaceIndex + 1);
    
      target_vx = -vxStr.toFloat(); //negate as wheels are mounted inversely
      target_omega = omegaStr.toFloat(); //negate as wheels are mounted inversely
    
      Serial.print("Parsed target_vx: ");
      Serial.print(target_vx);
      Serial.print(" | target_omega: ");
      Serial.println(target_omega);
    } else {
      Serial.println("Malformed input. Could not parse target velocities.");
    }


  }
  

  static bool first_run = true;
  
  unsigned long now = millis();
  
  if (now - motor_board.prev_encoder_time >= SENSOR_INTERVAL) {
    first_run = motor_board.run_encoders(first_run,now);
    sendStatus();
  }


  if (now-imu.prev_imu_time >= IMU_INTERVAL) {
    cachedIMU = imu.get_reading(now);
  }

  if (now - motor_board.prev_control_time >= CONTROL_INTERVAL) {
    float dt_control = (now  - motor_board.prev_control_time) / 1000.0;
    motor_board.run_control(target_vx,target_omega,dt_control,now);
  }
}




void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  if (myData.z && !last_z_state)
  {
    manual_mode = !manual_mode;
  }

  last_z_state = myData.z;
}
 


void sendStatus() {
  Serial.print("$");
  Serial.print(motor_board.get_encoder1());
  Serial.print(" ");
  Serial.print(motor_board.get_encoder2());
  
  Serial.print(" ");
  Serial.print(motor_board.get_rpm1());
  Serial.print(" ");
  Serial.print(motor_board.get_rpm2());

  Serial.print(" ");
  Serial.print(motor_board.get_current1());
  Serial.print(" ");
  Serial.print(motor_board.get_current2());

  
  Serial.print(" ");
  Serial.print(motor_board.get_battery_voltage());

  Serial.print(" ");
  Serial.print(motor_board.get_battery_power());

  Serial.print(" ");
  Serial.print(cachedIMU.acc_x);
  Serial.print(" ");
  Serial.print(cachedIMU.acc_y);
  Serial.print(" ");
  Serial.print(cachedIMU.acc_z);
  Serial.print(" ");
  Serial.print(cachedIMU.omega_x);
  Serial.print(" ");
  Serial.print(cachedIMU.omega_y);
  Serial.print(" ");
  Serial.print(cachedIMU.omega_z);
  Serial.print(" ");
  Serial.println(cachedIMU.temp);
}