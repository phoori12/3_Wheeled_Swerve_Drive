#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Module 1
#define INA_1 41
#define INB_1 40
#define PWM_1 37
#define prox_1 30
#define encA1 2
#define encB1 3

// Module 2
#define INA_2 39
#define INB_2 38
#define PWM_2 36
#define prox_2 31
#define encA2 4
#define encB2 5

// Module 3
#define INA_3 35
#define INB_3 34
#define PWM_3 33
#define prox_3 32
#define encA3 6
#define encB3 7
#define SW_Red 10
#define SW_Yel 11
#define SW_Bla 12

//////////////////////////////// GYRO Shit ////////////////////////////////
#include <BNO055_support.h>
#include <Wire.h>

// This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; // Structure to hold the Euler data

unsigned long lastTime_gyroRead = 0;
float gyro_now = 0;

float gyro_accept = 0.4f; //0.5f
float gyro_offset = 0.00f;
float gyro_localizeMargin = 3.00f;

float readGyro();
float degToRad(float val);
float radToDeg(float val);
void stopAll2();
void degAdj1_posCon();
void degAdj2_posCon();
void degAdj3_posCon();
void priorityDegPosCon(int pulse1, int pulse2, int pulse3);
void swerveDrive(float spd, float dir, float omega, bool rotateOnly = false);
void setDegSwerve(float deg1, float deg2, float deg3, float v1, float v2, float v3);
void spinCCW();
void homeTheta();
void stopAll();
void spin_drive(int mtr, int spd);
void ENCA1_Read();
void ENCB1_Read();
void ENCA2_Read();
void ENCB2_Read();
void ENCA3_Read();
void ENCB3_Read();
void sendCmd(float spd1, float spd2, float spd3);
void headingControl(float spd, float course, float set_head);
void rotationControl(float spd, float set_head);
void moveWithDelay(float spd, float dir, float omega, int duration);
void getRobotPosition();
void p2ptrack(float set_x, float set_y, float set_head, bool viaMode = false);
float closestAngle(float a, float b);
void tuneSwerveKit(int swerveNo, float setpoint_deg, float kp, float ki, float kd);
void printPos();
float mapDeg(float deg);

String waitForInput();
void straightCmdExec();
void circularCmdExec();
void rectangleCmdExec();
void triangleCmdExec();

// path fuctions //
void straightMove(float d);
void triangleMove(float b, float h, bool r);
void rectangleMove(float w, float h);
void fullCircleMove(float r, bool lh);
void halfCircleMove(float r, bool lh);

volatile long ENC1_Count = 0;
volatile long ENC2_Count = 0;
volatile long ENC3_Count = 0;

// Degree Adjustor Position Control Variables //
float pulsePerDeg = 11.377778;

// PID Variables // PID ชุดหมุน
const float p_kp1 = 7.35f, p_ki1 = 0.05f, p_kd1 = 1.40f; // kp 8.00f
const float p_kp2 = 7.45f, p_ki2 = 0.15f, p_kd2 = 1.32f;
const float p_kp3 = 7.50f, p_ki3 = 0.03f, p_kd3 = 1.00f;
////////////////////////////////

float p_edit1, p_error1 = 0, p_preverror1 = 0, p_p1 = 0, p_i1 = 0, p_d1 = 0;
float p_edit2, p_error2 = 0, p_preverror2 = 0, p_p2 = 0, p_i2 = 0, p_d2 = 0;
float p_edit3, p_error3 = 0, p_preverror3 = 0, p_p3 = 0, p_i3 = 0, p_d3 = 0;

volatile int swerve_deg1 = 0;
volatile int swerve_deg2 = 0;
volatile int swerve_deg3 = 0;

volatile int prev_swerve_deg1 = swerve_deg1;
volatile int prev_swerve_deg2 = swerve_deg2;
volatile int prev_swerve_deg3 = swerve_deg3;

volatile bool deg1Flag = false;
volatile bool deg2Flag = false;
volatile bool deg3Flag = false;
volatile bool stopFlag = false;

volatile bool negMul_1 = false;
volatile bool negMul_2 = false;
volatile bool negMul_3 = false;

// Swerve Offset and Deg Conversion Variables //
int swerve_off1 = 1792;  // 1800
int swerve_off2 = -1036; // -1100
int swerve_off3 = 436;   // 400

int swerve_right1 = -278;  //-300
int swerve_right2 = -3163; // -3200
int swerve_right3 = -1706; // 2500
volatile float prev_deg1, prev_deg2, prev_deg3;

int marginError = 25;

const float DegToPulseConst = 23.3333; // 23.3333f
const float degToPulseConst_1 = (swerve_off1 - swerve_right1) / 90;
const float degToPulseConst_2 = (swerve_off2 - swerve_right2) / 90;
const float degToPulseConst_3 = (swerve_off3 - swerve_right3) / 90;

IntervalTimer subroutine_posCon1;
IntervalTimer subroutine_posCon2;
IntervalTimer subroutine_posCon3;
// IntervalTimer subroutine_printPos;

uint32_t readCountTime;
uint32_t sendSpeedTime;

// Localization Variables //
const float xCon = 0.0000484f; // To be tuned //
const float yCon = 0.0000469f; // To be tuned //
long od1_off = 0, od2_off = 0, od3_off = 0;
volatile long od1_now = 0, od2_now = 0, od3_now = 0; // tends to overflow
volatile long od1 = 0, od2 = 0, od3 = 0;
volatile float actual_deg1 = 0, actual_deg2 = 0, actual_deg3 = 0;
volatile long last_od1 = 0, last_od2 = 0, last_od3 = 0;
volatile float x_frame, y_frame, x_glob = 0, y_glob = 0;

///////////////////// P2P Control Vars /////////////////////
float mapgyro;
float d_i = 0;
const float s_kp = 10.0f, s_ki = 0.20f, s_kd = 4.0f;
const float h_kp = 2.50f, h_ki = 0.00f, h_kd = 1.25f; // PID Heading //
const float r_kp = 2.50f, r_ki = 0.00f, r_kd = 1.25f; // PID Rotation //
float dx, dy, dsm, s_error, d_s, s_edit, compensateTht;
float h_edit, h_error = 0, h_preverror = 0, h_p = 0, h_i = 0, h_d = 0;
float r_edit, r_error = 0, r_preverror = 0, r_p = 0, r_i = 0, r_d = 0;
long p2pTargetTime = 0;
////////////////////////////////////////////////////////////

#define MAX_SPD 10 // MAX RPM 70

union packed_int
{
  int32_t i;
  byte b[4];
} m1, m2, m3;

volatile long int x = 0;

union packed_uint
{
  long int num;
  byte b[4];
} num_rec;

void setup()
{
  // Gyro Init //
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // BNO RESET PIN

  Wire.begin();
  delay(2000);
  // Initialization of the BNO055
  BNO_Init(&myBNO); // Assigning the structure to hold information about the device

  // Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(10);

  Serial.begin(9600);
  Serial5.begin(115200); // motor 1
  Serial4.begin(115200); // motor 2
  Serial3.begin(115200); // motor 3

  // initialize device

  // Switches Init //
  pinMode(SW_Red, INPUT);
  pinMode(SW_Yel, INPUT);
  pinMode(SW_Bla, INPUT);

  // Swerve Dish Init //
  pinMode(prox_1, INPUT);
  pinMode(prox_2, INPUT);
  pinMode(prox_3, INPUT);
  pinMode(INA_1, OUTPUT);
  pinMode(INB_1, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  pinMode(INA_2, OUTPUT);
  pinMode(INB_2, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(INA_3, OUTPUT);
  pinMode(INB_3, OUTPUT);
  pinMode(PWM_3, OUTPUT);

  // Encoder Init //
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  pinMode(encA3, INPUT);
  pinMode(encB3, INPUT);
  attachInterrupt(encA1, ENCA1_Read, RISING);
  attachInterrupt(encB1, ENCB1_Read, RISING);
  attachInterrupt(encA2, ENCA2_Read, RISING);
  attachInterrupt(encB2, ENCB2_Read, RISING);
  attachInterrupt(encA3, ENCA3_Read, RISING);
  attachInterrupt(encB3, ENCB3_Read, RISING);

  // PWM Freq and Res Config //
  analogWriteFrequency(PWM_1, 10000);
  analogWriteFrequency(PWM_2, 10000);
  analogWriteFrequency(PWM_3, 10000);
  analogWriteResolution(10);

  // Manual Swerve Adjustments
  spin_drive(1, 0);
  spin_drive(2, 0);
  spin_drive(3, 0);
  // Serial.println("First wheel setup");
  while (digitalRead(SW_Bla) == 1) // 1st Swerve
  {
    if (digitalRead(SW_Yel) == 0 && digitalRead(SW_Red) == 1)
    {
      spin_drive(1, -700);
    }
    else if (digitalRead(SW_Yel) == 1 && digitalRead(SW_Red) == 0)
    {
      spin_drive(1, 700);
    }
    else
    {
      spin_drive(1, 0);
    }
  }
  // Serial.println("Second wheel setup");
  delay(1000);
  while (digitalRead(SW_Bla) == 1) // 2nd Swerve
  {
    if (digitalRead(SW_Yel) == 0 && digitalRead(SW_Red) == 1)
    {
      spin_drive(2, -700);
    }
    else if (digitalRead(SW_Yel) == 1 && digitalRead(SW_Red) == 0)
    {
      spin_drive(2, 700);
    }
    else
    {
      spin_drive(2, 0);
    }
  }
  // Serial.println("Third wheel setup");
  delay(1000);
  while (digitalRead(SW_Bla) == 1) // 3rd Swerve
  {
    if (digitalRead(SW_Yel) == 0 && digitalRead(SW_Red) == 1)
    {
      spin_drive(3, -700);
    }
    else if (digitalRead(SW_Yel) == 1 && digitalRead(SW_Red) == 0)
    {
      spin_drive(3, 700);
    }
    else
    {
      spin_drive(3, 0);
    }
  }
  delay(1000);
  // Serial.println("GO HOME");
  // Homing before start //
  while (digitalRead(SW_Bla) == 1)
  {
    if (digitalRead(SW_Yel) == 0)
    {
      spinCCW();
    }
    else
    {
      spin_drive(1, 0);
      spin_drive(2, 0);
      spin_drive(3, 0);
    }
    if (digitalRead(SW_Red) == 0)
    {
      homeTheta();
      break;
    }
  }
  stopAll2();

  // Start Sub routine (Swerve Dish Position Control) ////////////////
  subroutine_posCon1.begin(degAdj1_posCon, 1000);
  subroutine_posCon2.begin(degAdj2_posCon, 1000);
  subroutine_posCon3.begin(degAdj3_posCon, 1000);
  // subroutine_printPos.begin(printPos, 10000);

  setDegSwerve(0, 0, 0, 0, 0, 0);
  delay(1000);

  // Serial.println("press red");

  // Get Gyro Offset ..
  bno055_read_euler_hrp(&myEulerData);
  float gyro_read = mapDeg(float(myEulerData.h) / 16.00);
  gyro_offset = gyro_read;
  Serial.print("Heading(Yaw): ");               // To read out the Heading (Yaw)
  Serial.println(float(myEulerData.h) / 16.00); // Convert to degrees
  // stopFlag = true;
  // stopAll2();
  Serial.println("Ready to recieve cmd");
}

long lasttime1 = 0, lasttime2 = 0, lasttime3 = 0;
uint32_t looptime = 1;

long lasttime_shit = 0;

uint32_t localizeTime = 0;
bool safeMode = true;

// testing heading
float set_head_values[] = {15, 30, 45, 60, 75, 90}; 
int num_values = sizeof(set_head_values) / sizeof(set_head_values[0]);
bool programStarted = false; 
int targetIndex = 0;

void loop()
{
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  tuneSwerveKit(1, 90, 2, 0, 0); 
  

  // Serial.println("1. Straight Path");
  // Serial.println("2. Triangle");
  // Serial.println("3. Rectangle / Square");
  // Serial.println("4. Circular");
  // String cmd = waitForInput();

  // switch (cmd.toInt())
  // {
  // case 1:
  //   straightCmdExec();
  //   break;
  // case 2:
  //   triangleCmdExec();
  //   break;
  // case 3:
  //   rectangleCmdExec();
  //   break;
  // case 4:
  //   circularCmdExec();
  //   break;
  // default:
  //   Serial.println("Invalid Input");
  //   return;
  // }

  // Serial.println("CMD Completed");

  // // setDegSwerve(0, 0, 0, 0, 0, 0);
  // // delay(1000);
  // while (1)
  // {
  //   stopFlag = true;
  //   stopAll2();
  // }
}

void testheading()
{
  if (digitalRead(SW_Bla) == 0) {
    programStarted = true; // Set the flag to start the program
    //Serial.println("In");
  }

  // Run the program if the switch is pressed
  if (programStarted == true) {
    if (targetIndex < num_values) {
      rotationControl(10, set_head_values[targetIndex]);
      //Serial.print("Target ");
      //Serial.println(set_head_values[targetIndex]);
    } else {
      Serial.println("All targets reached");
      programStarted = false; // Optionally reset the flag to stop the program
      // Reset targetIndex to 0 if you want to restart the sequence on next switch press
      targetIndex = 0;
    }
  }
}
void straightMove(float d)
{
  setDegSwerve(90, 90, 90, 0, 0, 0);
  delay(500);
  while (digitalRead(SW_Red) == 1)
    ;
  delay(1000);
  bno055_read_euler_hrp(&myEulerData);
  float gyro_read = mapDeg(float(myEulerData.h) / 16.00);
  gyro_offset = gyro_read;
  p2ptrack(0, d, 0);
  stopAll2();
}

void triangleMove(float b, float h, bool r)
{
  if (r)
  {
    setDegSwerve(90, 90, 90, 0, 0, 0);
  }
  else
  {
    setDegSwerve(0, 0, 0, 0, 0, 0);
  }
  delay(500);
  while (digitalRead(SW_Red) == 1)
    ;
  delay(1000);
  bno055_read_euler_hrp(&myEulerData);
  float gyro_read = mapDeg(float(myEulerData.h) / 16.00);
  gyro_offset = gyro_read;
  if (r)
  {
    p2ptrack(0, h, 0); // 1 2 0
    stopAll2();
    delay(500);
    while (digitalRead(SW_Red) == 1)
      ;
    delay(1000);
    p2ptrack(b, 0, 0); // 2 0 0
    stopAll2();
    delay(500);
    while (digitalRead(SW_Red) == 1)
      ;
    delay(1000);
    p2ptrack(0, 0, 0);
    stopAll2();
    delay(500);
  }
  else
  {
    p2ptrack(b, 0, 0); // 1 2 0
    stopAll2();
    delay(500);
    while (digitalRead(SW_Red) == 1)
      ;
    delay(1000);
    p2ptrack(b / 2, h, 0); // 2 0 0
    stopAll2();
    delay(500);
    while (digitalRead(SW_Red) == 1)
      ;
    delay(1000);
    p2ptrack(0, 0, 0);
    stopAll2();
    delay(500);
  }
}

void rectangleMove(float w, float h)
{
  setDegSwerve(90, 90, 90, 0, 0, 0);
  delay(500);
  while (digitalRead(SW_Red) == 1)
    ;
  delay(1000);
  bno055_read_euler_hrp(&myEulerData);
  float gyro_read = mapDeg(float(myEulerData.h) / 16.00);
  gyro_offset = gyro_read;
  p2ptrack(0, h, 0); // 1 2 0
  stopAll2();
  delay(500);
  while (digitalRead(SW_Red) == 1)
    ;
  delay(1000);
  p2ptrack(w, h, 0); // 2 0 0
  stopAll2();
  delay(500);
  while (digitalRead(SW_Red) == 1)
    ;
  delay(1000);
  p2ptrack(w, 0, 0);
  stopAll2();
  delay(500);
  while (digitalRead(SW_Red) == 1)
    ;
  delay(1000);
  p2ptrack(0, 0, 0);
  stopAll2();
}

void fullCircleMove(float r, bool lh)
{
  setDegSwerve(0, 0, 0, 0, 0, 0);
  delay(500);
  while (digitalRead(SW_Red) == 1)
    ;
  delay(1000);
  bno055_read_euler_hrp(&myEulerData);
  float gyro_read = mapDeg(float(myEulerData.h) / 16.00);
  gyro_offset = gyro_read;
  if (lh)
  {
    for (int i = 0; i <= 360; i += 20)
    {
      float x = ceil(r * cos(degToRad(i)) * 100) / 100;
      float y = ceil(r * sin(degToRad(i)) * 100) / 100;
      if (i == 0 || i == 360)
      {
        p2ptrack(x, y, 0, false);
      }
      else
      {
        p2ptrack(x, y, 0, true);
      }
    }
  }
  else
  {
    for (int i = 0; i <= 360; i += 20)
    {
      float x = ceil(r * cos(degToRad(i)) * 100) / 100;
      float y = ceil(r * sin(degToRad(i)) * 100) / 100;
      if (i == 0 || i == 360)
      {
        p2ptrack(x, y, 0, false);
      }
      else
      {
        p2ptrack(x, y, i, true);
      }
    }
  }
}

void halfCircleMove(float r, bool lh)
{
  setDegSwerve(0, 0, 0, 0, 0, 0);
  delay(500);
  while (digitalRead(SW_Red) == 1)
    ;
  delay(1000);
  bno055_read_euler_hrp(&myEulerData);
  float gyro_read = mapDeg(float(myEulerData.h) / 16.00);
  gyro_offset = gyro_read;
  if (lh)
  {
    for (int i = 0; i <= 180; i += 20)
    {
      float x = ceil(r * cos(degToRad(i)) * 100) / 100;
      float y = ceil(r * sin(degToRad(i)) * 100) / 100;
      if (i == 0 || i == 360)
      {
        p2ptrack(x, y, 0, false);
      }
      else
      {
        p2ptrack(x, y, 0, true);
      }
    }
  }
  else
  {
    for (int i = 0; i <= 180; i += 20)
    {
      float x = ceil(r * cos(degToRad(i)) * 100) / 100;
      float y = ceil(r * sin(degToRad(i)) * 100) / 100;
      if (i == 0 || i == 360)
      {
        p2ptrack(x, y, 0, false);
      }
      else
      {
        p2ptrack(x, y, i, true);
      }
    }
  }
}

void p2ptrack(float set_x, float set_y, float set_head, bool viaMode = false)
{
  static volatile float s_prev_error = 0.0f;
  static bool onPoint = false;
  static float theta = 0;
  set_head = -set_head;

  if (abs(set_head) > 180.0f)
  {
    int signum = 0;
    if (set_head > 0)
      signum = 1;
    if (set_head < 0)
      signum = -1;
    if (set_head == 0)
      signum = 0;

    set_head = -(signum * 360) + set_head;
  }

  while (1)
  {
    getRobotPosition();
    dx = set_x - x_glob;
    dy = set_y - y_glob;
    dsm = sqrt(pow(dx, 2) + pow(dy, 2));

    s_error = dsm;
    d_i += s_error;
    d_i = constrain(d_i, -1000, 1000);
    d_s = s_error - s_prev_error;
    s_prev_error = s_error;

    float gyro_pos = readGyro();
    if (abs(abs(gyro_pos) - set_head) > gyro_accept)
    {
      h_error = gyro_pos - set_head;
    }
    else
    {
      h_error = 0;
    }
    float h_checkCompensate = closestAngle(h_preverror, h_error);
    h_error = h_preverror + h_checkCompensate;

    h_p = h_kp * h_error;
    h_d = (h_error - h_preverror) * h_kd;
    h_preverror = h_error;
    h_edit = h_p + h_d;
    // Serial.println(h_edit);

    if (dx != 0 || dy != 0)
    {
      theta = atan2(dy, dx) * (180 / PI);
    }

    // if cz > pi : cz = 2pi * cz
    // if cz < -pi : cz = 2pi + cz

    mapgyro = gyro_pos;
    if (mapgyro > 0)
    {
      mapgyro = fmod(mapgyro, 360);
    }
    else if (mapgyro < 0)
    {
      mapgyro = fmod(abs(mapgyro), 360 * -1);
    }

    s_edit = (s_error * s_kp) + (d_i * s_ki) + (s_kd * d_s);
    if (set_head >= 0)
    {
      compensateTht = theta + mapgyro;
    }
    else
    {
      compensateTht = theta - mapgyro;
    }
    // compensateTht = theta;
    if ((abs(dx) <= 0.05 && abs(dy) <= 0.05) && abs(h_error) <= gyro_accept + gyro_localizeMargin)
    {

      if (onPoint == false)
      {
        p2pTargetTime = millis();
      }
      onPoint = true;
      if (millis() - p2pTargetTime > 500 || viaMode)
      {
        sendCmd(0, 0, 0);
        break;
      }
    }
    else
    {
      onPoint = false;
    }
    if (s_edit > MAX_SPD)
    {
      s_edit = MAX_SPD;
    }
    else if (s_edit < -MAX_SPD)
    {
      s_edit = -MAX_SPD;
    }

    // Serial.print(h_error);
    // Serial.print("\t");
    // Serial.print(-h_edit);
    // Serial.print("\t");
    // Serial.println(gyro_pos);
    // Serial.print(theta);
    // Serial.print("\t");
    // Serial.print(s_edit);
    // Serial.print("\t");
    // Serial.print(compensateTht);
    // Serial.print("\t");
    // Serial.println(-h_edit);
    // Serial.print("\t");
    // Serial.print(x_glob);
    // Serial.print("\t");
    // Serial.println(y_glob);
    swerveDrive(s_edit, compensateTht, -h_edit);
  }
}

void getRobotPosition()
{
  // Get Current Rotations
  serialEvent5();
  serialEvent4();
  serialEvent3();
  // check overflow and get motor rotations
  if (abs(last_od1 - od1_now) < 1000000L)
  {
    od1 = od1_now - last_od1 - od1_off; // vw1
  }

  if (abs(last_od2 - od2_now) < 1000000L)
  {
    od2 = od2_now - last_od2 - od2_off; // vw2
  }

  if (abs(last_od3 - od3_now) < 1000000L)
  {
    od3 = -(od3_now - last_od3 - od3_off); // vw3
  }
  last_od1 = od1_now;
  last_od2 = od2_now;
  last_od3 = od3_now;

  // Localization Equation Here
  static float vx, vy, v1x, v2x, v3x, v1y, v2y, v3y;
  v1x = od1 * cos(degToRad(actual_deg1));
  v1y = od1 * sin(degToRad(actual_deg1));

  v2x = od2 * cos(degToRad(actual_deg2));
  v2y = od2 * sin(degToRad(actual_deg2));

  v3x = od3 * cos(degToRad(actual_deg3));
  v3y = od3 * sin(degToRad(actual_deg3));

  vx = (v1x + v2x + v3x) * 0.00142f;
  vy = (v1y + v2y + v3y) * 0.00142f;

  x_frame = vx * xCon;
  y_frame = vy * yCon;
  float gyro_pos = -readGyro();
  x_glob += (x_frame * cos(degToRad(gyro_pos)) - y_frame * sin(degToRad(gyro_pos)));
  y_glob += (x_frame * sin(degToRad(gyro_pos)) + y_frame * cos(degToRad(gyro_pos)));

  // Localize plot uncomment here // 
  Serial.print(x_glob);
  Serial.print(",");
  Serial.print(y_glob);
  Serial.print(",");
  Serial.println(gyro_pos);
}

void moveWithDelay(float spd, float dir, float head, int duration)
{
  // TODO
}

float prev_thet1 = 0;
float prev_thet2 = 0;
float prev_thet3 = 0;

void swerveDrive(float spd, float dir, float omega, bool rotateOnly = false)
{
  if (millis() - sendSpeedTime > 10)
  {
    sendSpeedTime = millis();
    float vx, vy;
    float vx1, vy1, vx2, vy2, vx3, vy3;
    float vw1, vw2, vw3, thet1, thet2, thet3;
    vx = spd * cos(degToRad(dir));
    vy = spd * sin(degToRad(dir));
    vx1 = omega * 0.255 + vx;
    vy1 = vy;
    vx2 = -omega * 0.255 * sin(degToRad(30)) + vx;
    vy2 = -omega * 0.255 * sin(degToRad(60)) + vy;
    vx3 = -omega * 0.255 * sin(degToRad(30)) + vx;
    vy3 = omega * 0.255 * sin(degToRad(60)) + vy;

    // TODO Contrain shit
    vw1 = sqrt(pow(vx1, 2) + pow(vy1, 2));
    vw2 = sqrt(pow(vx2, 2) + pow(vy2, 2));
    vw3 = -sqrt(pow(vx3, 2) + pow(vy3, 2));
    if (vw1 > spd)
    {
      vw1 = spd;
    }
    else if (vw1 < -spd)
    {
      vw1 = -spd;
    }

    if (vw2 > spd)
    {
      vw2 = spd;
    }
    else if (vw2 < -spd)
    {
      vw2 = -spd;
    }

    if (vw3 > spd)
    {
      vw3 = spd;
    }
    else if (vw3 < -spd)
    {
      vw3 = -spd;
    }

    if (vx1 == 0 && vy1 == 0)
    {
      thet1 = prev_thet1;
    }
    else
    {
      thet1 = radToDeg(atan2(vy1, vx1));
    }

    if (vx2 == 0 && vy2 == 0)
    {
      thet2 = prev_thet2;
    }
    else
    {
      thet2 = radToDeg(atan2(vy2, vx2));
    }

    if (vx3 == 0 && vy3 == 0)
    {
      thet3 = prev_thet3;
    }
    else
    {
      thet3 = radToDeg(atan2(vy3, vx3));
    }

    if (negMul_1)
    {
      vw1 = -vw1;
    }

    if (negMul_2)
    {
      vw2 = -vw2;
    }

    if (negMul_3)
    {
      vw3 = -vw3;
    }

    if (rotateOnly)
    {
      setDegSwerve(0, 45, 135, vw1, vw2, vw3);
    }
    else
    {
      setDegSwerve(thet1, thet2, thet3, vw1, vw2, vw3);
    }
    prev_thet1 = actual_deg1;
    prev_thet2 = actual_deg2;
    prev_thet3 = actual_deg3;
  }
}

void priorityDegPosCon(int pulse1, int pulse2, int pulse3)
{
  long targetTime = 500;
  bool atTarget = false;
  uint32_t last = millis();

  swerve_deg1 = pulse1;
  swerve_deg2 = pulse2;
  swerve_deg3 = pulse3;

  while (true)
  {
    if (millis() - last >= looptime)
    {
      last = millis();

      if (abs(ENC1_Count - swerve_deg1) > marginError)
      {
        p_error1 = swerve_deg1 - ENC1_Count;
      }
      else
      {
        p_error1 = 0;
      }

      p_p1 = p_kp1 * p_error1;
      p_i1 += p_error1;
      p_i1 = constrain(p_i1, -2046, 2046);
      p_d1 = (p_error1 - p_preverror1) * p_kd1;
      p_preverror1 = p_error1;
      p_edit1 = p_p1 + (p_i1 * p_ki1) + p_d1;
      if (p_edit1 > 1000)
      {
        p_edit1 = 1000;
      }
      else if (p_edit1 < -1000)
      {
        p_edit1 = -1000;
      }
      spin_drive(1, p_edit1);

      if (abs(ENC2_Count - swerve_deg2) > marginError)
      {
        p_error2 = swerve_deg2 - ENC2_Count;
      }
      else
      {
        p_error2 = 0;
      }

      p_p2 = p_kp2 * p_error2;
      p_i2 += p_error2;
      p_i2 = constrain(p_i2, -2046, 2046);
      p_d2 = (p_error2 - p_preverror2) * p_kd2;
      p_preverror2 = p_error2;
      p_edit2 = p_p2 + (p_i2 * p_ki2) + p_d2;
      if (p_edit2 > 1000)
      {
        p_edit2 = 1000;
      }
      else if (p_edit2 < -1000)
      {
        p_edit2 = -1000;
      }
      spin_drive(2, p_edit2);

      if (abs(ENC3_Count - swerve_deg3) > marginError)
      {
        p_error3 = swerve_deg3 - ENC3_Count;
      }
      else
      {
        p_error3 = 0;
      }

      p_p3 = p_kp3 * p_error3;
      p_i3 += p_error3;
      p_i3 = constrain(p_i3, -2046, 2046);
      p_d3 = (p_error3 - p_preverror3) * p_kd3;
      p_preverror3 = p_error3;
      p_edit3 = p_p3 + (p_i3 * p_ki3) + p_d3;
      if (p_edit3 > 1000)
      {
        p_edit3 = 1000;
      }
      else if (p_edit3 < -1000)
      {
        p_edit3 = -1000;
      }
      spin_drive(3, p_edit3);
    }

    // Position Check
    if (abs(p_error1) < marginError && abs(p_error2) < marginError && abs(p_error3) < marginError)
    {
      if (atTarget == false)
      {
        targetTime = millis();
      }
      atTarget = true;
      if (millis() - targetTime > 500)
      {
        stopAll2();
        stopFlag = false;
        break;
      }
    }
    else
    {
      atTarget = false;
    }
  }
}

float degToRad(float val)
{
  return val * DEG_TO_RAD;
}

float radToDeg(float val)
{
  return val * RAD_TO_DEG;
}

void stopAll2()
{
  spin_drive(1, 0);
  spin_drive(2, 0);
  spin_drive(3, 0);
  sendSpeedTime = millis();
  if (millis() - sendSpeedTime > 10)
  {
    sendSpeedTime = millis();
    sendCmd(0, 0, 0);
  }
}

void degAdj1_posCon()
{
  if (millis() - lasttime1 >= looptime && !stopFlag)
  {
    lasttime1 = millis();
    if (abs(ENC1_Count - swerve_deg1) > marginError)
    {
      p_error1 = swerve_deg1 - ENC1_Count;
    }
    else
    {
      p_error1 = 0;
    }

    p_p1 = p_kp1 * p_error1;
    p_i1 += p_error1;
    p_i1 = constrain(p_i1, -2046, 2046);
    p_d1 = (p_error1 - p_preverror1) * p_kd1;
    p_preverror1 = p_error1;
    p_edit1 = p_p1 + (p_i1 * p_ki1) + p_d1;
    if (p_edit1 > 1000)
    {
      p_edit1 = 1000;
    }
    else if (p_edit1 < -1000)
    {
      p_edit1 = -1000;
    }
    spin_drive(1, p_edit1);
  }
}

void degAdj2_posCon()
{
  if (millis() - lasttime2 >= looptime && !stopFlag)
  {
    lasttime2 = millis();
    if (abs(ENC2_Count - swerve_deg2) > marginError)
    {
      p_error2 = swerve_deg2 - ENC2_Count;
    }
    else
    {
      p_error2 = 0;
    }

    p_p2 = p_kp2 * p_error2;
    p_i2 += p_error2;
    p_i2 = constrain(p_i2, -2046, 2046);
    p_d2 = (p_error2 - p_preverror2) * p_kd2;
    p_preverror2 = p_error2;
    p_edit2 = p_p2 + (p_i2 * p_ki2) + p_d2;
    if (p_edit2 > 1000)
    {
      p_edit2 = 1000;
    }
    else if (p_edit2 < -1000)
    {
      p_edit2 = -1000;
    }
    spin_drive(2, p_edit2);
  }
}

void degAdj3_posCon()
{
  if (millis() - lasttime3 >= looptime && !stopFlag)
  {
    lasttime3 = millis();
    if (abs(ENC3_Count - swerve_deg3) > marginError)
    {
      p_error3 = swerve_deg3 - ENC3_Count;
    }
    else
    {
      p_error3 = 0;
    }

    p_p3 = p_kp3 * p_error3;
    p_i3 += p_error3;
    p_i3 = constrain(p_i3, -2046, 2046);
    p_d3 = (p_error3 - p_preverror3) * p_kd3;
    p_preverror3 = p_error3;
    p_edit3 = p_p3 + (p_i3 * p_ki3) + p_d3;
    if (p_edit3 > 1000)
    {
      p_edit3 = 1000;
    }
    else if (p_edit3 < -1000)
    {
      p_edit3 = -1000;
    }
    spin_drive(3, p_edit3);
  }
}

float closestAngle(float a, float b)
{
  float dir = fmod(b, 360) - fmod(a, 360);
  return mapDeg(dir);
}

float mapDeg(float deg)
{
  if (abs(deg) > 180.0f)
  {
    int signum = 0;
    if (deg > 0)
      signum = 1;
    if (deg < 0)
      signum = -1;
    if (deg == 0)
      signum = 0;

    deg = -(signum * 360) + deg;
  }

  return deg;
}

void setDegSwerve(float deg1, float deg2, float deg3, float v1, float v2, float v3)
{
  stopFlag = false;

  // Closet Angle
  // negMul
  float checkdeg1 = closestAngle(prev_deg1, deg1);
  float checkdeg2 = closestAngle(prev_deg2, deg2);
  float checkdeg3 = closestAngle(prev_deg3, deg3);

  float checkdeg1_flipped = closestAngle(prev_deg1, deg1 + 180.0f);
  float checkdeg2_flipped = closestAngle(prev_deg2, deg2 + 180.0f);
  float checkdeg3_flipped = closestAngle(prev_deg3, deg3 + 180.0f);

  if (abs(checkdeg1) <= abs(checkdeg1_flipped))
  {
    negMul_1 = false;
    deg1 = prev_deg1 + checkdeg1;
  }
  else if (abs(checkdeg1) >= abs(checkdeg1_flipped))
  {
    negMul_1 = true;
    deg1 = prev_deg1 + checkdeg1_flipped;
  }

  if (abs(checkdeg2) <= abs(checkdeg2_flipped))
  {
    negMul_2 = false;
    deg2 = prev_deg2 + checkdeg2;
  }
  else if (abs(checkdeg2) >= abs(checkdeg2_flipped))
  {
    negMul_2 = true;
    deg2 = prev_deg2 + checkdeg2_flipped;
  }

  if (abs(checkdeg3) <= abs(checkdeg3_flipped))
  {
    negMul_3 = false;
    deg3 = prev_deg3 + checkdeg3;
  }
  else if (abs(checkdeg3) >= abs(checkdeg3_flipped))
  {
    negMul_3 = true;
    deg3 = prev_deg3 + checkdeg3_flipped;
  }

  if (safeMode)
  {
    if (deg1 >= 360)
    {
      if (negMul_1)
      {
        deg1 = 360 - (180 + checkdeg1_flipped);
        negMul_1 = false;
      }
      else
      {
        deg1 = 360 - (180 + checkdeg1);
        negMul_1 = true;
      }
    }
    else if (deg1 <= -360)
    {
      if (negMul_1)
      {
        deg1 = -360 + (180 - checkdeg1_flipped);
        negMul_1 = false;
      }
      else
      {
        deg1 = -360 + (180 - checkdeg1);
        negMul_1 = true;
      }
    }
    /////////////////
    if (deg2 >= 360)
    {
      if (negMul_2)
      {
        deg2 = 360 - (180 + checkdeg2_flipped);
        negMul_2 = false;
      }
      else
      {
        deg2 = 360 - (180 + checkdeg2);
        negMul_2 = true;
      }
    }
    else if (deg2 <= -360)
    {
      if (negMul_2)
      {
        deg2 = -360 + (180 - checkdeg2_flipped);
        negMul_2 = false;
      }
      else
      {
        deg2 = -360 + (180 - checkdeg2);
        negMul_2 = true;
      }
    }
    //////////////////////
    if (deg3 >= 360)
    {
      if (negMul_3)
      {
        deg3 = 360 - (180 + checkdeg3_flipped);
        negMul_3 = false;
      }
      else
      {
        deg3 = 360 - (180 + checkdeg3);
        negMul_3 = true;
      }
    }
    else if (deg3 <= -360)
    {
      if (negMul_3)
      {
        deg3 = -360 + (180 - checkdeg3_flipped);
        negMul_3 = false;
      }
      else
      {
        deg3 = -360 + (180 - checkdeg3);
        negMul_3 = true;
      }
    }
  }
  actual_deg1 = deg1;
  actual_deg2 = deg2;
  actual_deg3 = deg3;

  swerve_deg1 = swerve_off1 - (deg1 * degToPulseConst_1);
  swerve_deg2 = swerve_off2 - (deg2 * degToPulseConst_2);
  swerve_deg3 = swerve_off3 - (deg3 * degToPulseConst_3);

  float gradUnterscheid_1 = abs(abs(prev_swerve_deg1) - abs(swerve_deg1));
  float gradUnterscheid_2 = abs(abs(prev_swerve_deg2) - abs(swerve_deg2));
  float gradUnterscheid_3 = abs(abs(prev_swerve_deg3) - abs(swerve_deg3));

  if (gradUnterscheid_1 > 1050 || gradUnterscheid_2 > 1050 || gradUnterscheid_3 > 1050) // 1050
  {
    sendCmd(0, 0, 0);
    stopFlag = true;
    priorityDegPosCon(swerve_deg1, swerve_deg2, swerve_deg3);
  }
  else
  {
    stopFlag = false;
    sendCmd(v1, v2, v3);
  }
  // sendCmd(v1, v2, v3);
  prev_swerve_deg1 = swerve_deg1;
  prev_swerve_deg2 = swerve_deg2;
  prev_swerve_deg3 = swerve_deg3;
  prev_deg1 = deg1;
  prev_deg2 = deg2;
  prev_deg3 = deg3;
  // Serial.print(negMul_1);
  // Serial.print("\t");
  // Serial.println(deg1);
}

void spinCCW()
{
  spin_drive(1, -400);
  spin_drive(2, -400);
  spin_drive(3, -400);
}

void homeTheta()
{
  while (digitalRead(prox_1) == 0)
  {
    spin_drive(1, 650);
  }
  spin_drive(1, -200);
  delay(500);
  while (digitalRead(prox_1) == 0)
  {
    spin_drive(1, 200);
  }
  spin_drive(1, 0);
  /////////////////////
  while (digitalRead(prox_2) == 0)
  {
    spin_drive(2, 650);
  }
  spin_drive(2, -350);
  delay(500);
  while (digitalRead(prox_2) == 0)
  {
    spin_drive(2, 350);
  }
  spin_drive(2, 0);
  ////////////////////
  while (digitalRead(prox_3) == 0)
  {
    spin_drive(3, 650);
  }
  spin_drive(3, -200);
  delay(500);
  while (digitalRead(prox_3) == 0)
  {
    spin_drive(3, 200);
  }
  spin_drive(3, 0);
  delay(500);
  ENC1_Count = 0;
  ENC2_Count = 0;
  ENC3_Count = 0;
  delay(100);
}

void stopAll()
{
  while (1)
  {
    spin_drive(1, 0);
    spin_drive(2, 0);
    spin_drive(3, 0);
  }
}

void spin_drive(int mtr, int spd)
{
  if (mtr == 1)
  {
    if (spd > 0)
    { // CW
      digitalWrite(INA_1, 0);
      digitalWrite(INB_1, 1);
      analogWrite(PWM_1, spd);
    }
    else if (spd < 0)
    { // CCW
      digitalWrite(INA_1, 1);
      digitalWrite(INB_1, 0);
      analogWrite(PWM_1, -spd);
    }
    else
    {
      digitalWrite(INA_1, 1);
      digitalWrite(INB_1, 1);
      analogWrite(PWM_1, 0);
    }
  }
  else if (mtr == 2)
  {
    if (spd > 0)
    { // CW
      digitalWrite(INA_2, 0);
      digitalWrite(INB_2, 1);
      analogWrite(PWM_2, spd);
    }
    else if (spd < 0)
    { // CCW
      digitalWrite(INA_2, 1);
      digitalWrite(INB_2, 0);
      analogWrite(PWM_2, -spd);
    }
    else
    {
      digitalWrite(INA_2, 1);
      digitalWrite(INB_2, 1);
      analogWrite(PWM_2, 0);
    }
  }
  else if (mtr == 3)
  {
    if (spd > 0)
    {
      digitalWrite(INA_3, 0);
      digitalWrite(INB_3, 1);
      analogWrite(PWM_3, spd);
    }
    else if (spd < 0)
    {
      digitalWrite(INA_3, 1);
      digitalWrite(INB_3, 0);
      analogWrite(PWM_3, -spd);
    }
    else
    {
      digitalWrite(INA_3, 1);
      digitalWrite(INB_3, 1);
      analogWrite(PWM_3, 0);
    }
  }
  else
  {
    digitalWrite(INA_1, 1);
    digitalWrite(INB_1, 1);
    analogWrite(PWM_1, 0);
    digitalWrite(INA_2, 1);
    digitalWrite(INB_2, 1);
    analogWrite(PWM_2, 0);
    digitalWrite(INA_3, 1);
    digitalWrite(INB_3, 1);
    analogWrite(PWM_3, 0);
  }
}

void ENCA1_Read()
{
  if (digitalRead(encB1) == LOW)
  {
    ENC1_Count--;
  }
  else
  {
    ENC1_Count++;
  }
  // Serial.println(ENCL_Count);
}

void ENCB1_Read()
{
  if (digitalRead(encA1) == LOW)
  {
    ENC1_Count++;
  }
  else
  {
    ENC1_Count--;
  }
}

void ENCA2_Read()
{
  if (digitalRead(encB2) == LOW)
  {
    ENC2_Count--;
  }
  else
  {
    ENC2_Count++;
  }
  // Serial.println(ENCL_Count);
}

void ENCB2_Read()
{
  if (digitalRead(encA2) == LOW)
  {
    ENC2_Count++;
  }
  else
  {
    ENC2_Count--;
  }
}

void ENCA3_Read()
{
  if (digitalRead(encB3) == LOW)
  {
    ENC3_Count--;
  }
  else
  {
    ENC3_Count++;
  }
  // Serial.println(ENCL_Count);
}

void ENCB3_Read()
{
  if (digitalRead(encA3) == LOW)
  {
    ENC3_Count++;
  }
  else
  {
    ENC3_Count--;
  }
}

void sendCmd(float spd1, float spd2, float spd3)
{
  m1.i = spd1 * 100;
  m2.i = spd2 * 100;
  m3.i = spd3 * 100;
  const char cmd1[8] = {'#', 's', m1.b[3], m1.b[2], m1.b[1], m1.b[0], '\r', '\n'};
  const char cmd2[8] = {'#', 's', m2.b[3], m2.b[2], m2.b[1], m2.b[0], '\r', '\n'};
  const char cmd3[8] = {'#', 's', m3.b[3], m3.b[2], m3.b[1], m3.b[0], '\r', '\n'};
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial5.write(cmd1[i]);
    Serial4.write(cmd2[i]);
    Serial3.write(cmd3[i]);
  }
}

volatile long int fromSerial(volatile byte packet[])
{
  num_rec.b[0] = packet[5];
  num_rec.b[1] = packet[4];
  num_rec.b[2] = packet[3];
  num_rec.b[3] = packet[2];
  long int out = num_rec.num;
  return out;
}

void serialEvent5() // m1
{
  while (Serial5.available())
  {
    static volatile byte _buffer[8];
    static volatile int i = 0;
    _buffer[i] = Serial5.read();
    if (i == 0 && _buffer[0] != '#')
    {
      return;
    }
    if (_buffer[i - 7] == '#' && _buffer[i] == '\n')
    {
      od1_now = fromSerial(_buffer);
    }
    i++;
    if (i > 7)
      i = 0;
  }
}

void serialEvent4() // m2
{
  while (Serial4.available())
  {
    static volatile byte _buffer[8];
    static volatile int i = 0;
    _buffer[i] = Serial4.read();
    if (i == 0 && _buffer[0] != '#')
    {
      return;
    }
    if (_buffer[i - 7] == '#' && _buffer[i] == '\n')
    {
      od2_now = fromSerial(_buffer);
    }
    i++;
    if (i > 7)
      i = 0;
  }
}

void serialEvent3() // m3
{
  while (Serial3.available())
  {
    static volatile byte _buffer[8];
    static volatile int i = 0;
    _buffer[i] = Serial3.read();
    if (i == 0 && _buffer[0] != '#')
    {
      return;
    }
    if (_buffer[i - 7] == '#' && _buffer[i] == '\n')
    {
      od3_now = fromSerial(_buffer);
    }
    i++;
    if (i > 7)
      i = 0;
  }
}

float readGyro()
{
  if ((millis() - lastTime_gyroRead) >= 10) // To stream at 10 Hz without using additional timers
  {
    lastTime_gyroRead = millis();
    bno055_read_euler_hrp(&myEulerData);
    float gyro_read = mapDeg(float(myEulerData.h) / 16.00);
    // Serial.print("Heading(Yaw): ");       //To read out the Heading (Yaw)
    // Serial.println(gyro_offset - gyro_read);   //Convert to degrees
    return -(gyro_offset - gyro_read);
  }
}

void headingControl(float spd, float course, float set_head)
{
  float gyro_pos = readGyro();
  if (abs(gyro_pos - set_head) > gyro_accept)
  {
    h_error = gyro_pos - set_head;
  }
  else
  {
    h_error = 0;
  }
  h_p = h_kp * h_error;
  h_d = (h_error - h_preverror) * h_kd;
  h_preverror = h_error;
  h_edit = h_p + h_d;
  Serial.println(gyro_pos);
  swerveDrive(spd, course, -h_edit, 0);
}
void rotationControl(float spd, float set_head) 
{
  float gyro_pos = readGyro();
  //Serial.println(gyro_pos);

  if (abs(gyro_pos - set_head) > gyro_accept) {
    r_error = gyro_pos - set_head;
    //Serial.print("r_error = ");
    //Serial.println(r_error);
  } 
  else {
    r_error = 0;
  }
  r_p = r_kp * r_error;
  r_d = (r_error - r_preverror) * r_kd;
  r_preverror = r_error;
  r_edit = r_p + r_d;
  // Serial.println(r_edit);
  if (r_edit > 0) {
    r_edit = spd;
    //Serial.println("+spd");
  }
  if (r_edit < 0) {
    r_edit = -spd;
    //Serial.println("-spd");
  }
  if (gyro_pos == set_head) {
    stopAll2();
    //r_edit = 0;
    // Move to the next target
    Serial.print("Finish ");
    Serial.println(set_head);
    while (digitalRead(SW_Bla) == 1)
    {
      /* code */
    }
    targetIndex++ ;
    if (targetIndex >= num_values) {
      Serial.println("All targets reached");
    }
  }

  Serial.println(gyro_pos);
  setDegSwerve(0, 45, 135, -r_edit, r_edit, -r_edit);
}

// void rotationControl(float spd, float set_head)
// {
//   float gyro_pos = readGyro();
//   Serial.println(gyro_pos);
//   if (abs(gyro_pos - set_head ) > gyro_accept)
//   {
//     r_error = gyro_pos - set_head;
//     Serial.print("r_error = ");
//     Serial.println(r_error);
//   }
//   else
//   {
//     r_error = 0;
//   }
//   r_p = r_kp * r_error;
//   r_d = (r_error - r_preverror) * r_kd;
//   r_preverror = r_error;
//   r_edit = r_p + r_d;
//   //Serial.println(r_edit);
//   if (r_edit > 0) 
//   {
//     r_edit = spd;
//     Serial.println("+spd");
//   }
//   if (r_edit < 0) 
//   {
//     r_edit = -spd;
//     Serial.println("-spd");
//   }
//   if (gyro_pos == set_head)
//   {
//     r_edit = 0;
    
//   }
  
//   Serial.println(gyro_pos);
//   setDegSwerve(0, 45, 135, -r_edit, r_edit, -r_edit);
// }

void tuneSwerveKit(int swerveNo, float setpoint_deg, float kp, float ki, float kd)
{

  // const float p_kp1 = 4.35f, p_ki1 = 0.05f, p_kd1 = 1.40f; // kp 8.00f
  // const float p_kp2 = 7.45f, p_ki2 = 0.15f, p_kd2 = 1.32f;
  // const float p_kp3 = 7.50f, p_ki3 = 0.03f, p_kd3 = 1.00f;
  stopFlag = true;
  float edit, error = 0, preverror = 0, p = 0, i = 0, d = 0;

  long targetTime = 500;
  bool atTarget = false;
  uint32_t last = millis();

  if (swerveNo == 1)
  {
    swerve_deg1 = swerve_off1 - (setpoint_deg * degToPulseConst_1);
  }
  else if (swerveNo == 2)
  {
    swerve_deg2 = swerve_off2 - (setpoint_deg * degToPulseConst_2);
  }
  else
  {
    swerve_deg3 = swerve_off3 - (setpoint_deg * degToPulseConst_3);
  }

  while (digitalRead(SW_Red) == 1)
  {
    if (millis() - last >= looptime)
    {
      last = millis();

      if (swerveNo == 1)
      {
        if (abs(ENC1_Count - swerve_deg1) > marginError)
        {
          error = swerve_deg1 - ENC1_Count;
        }
        else
        {
          error = 0;
        }
      }
      else if (swerveNo == 2)
      {
        if (abs(ENC2_Count - swerve_deg2) > marginError)
        {
          error = swerve_deg2 - ENC2_Count;
        }
        else
        {
          error = 0;
        }
      }
      else
      {
        if (abs(ENC3_Count - swerve_deg3) > marginError)
        {
          error = swerve_deg3 - ENC3_Count;
        }
        else
        {
          error = 0;
        }
      }
      
      Serial.print(setpoint_deg);
      Serial.print(",");

      if (swerveNo == 1) {
        float deg = - (ENC1_Count - swerve_off1) / degToPulseConst_1;
        Serial.println(deg);
      } else if (swerveNo == 2) {
        float deg = - (ENC2_Count - swerve_off2) / degToPulseConst_2;
        Serial.println(deg);
      } else {
        float deg = - (ENC3_Count - swerve_off3) / degToPulseConst_3;
        Serial.println(deg);
      }
     
      //  swerve_deg1 = swerve_off1 - (deg1 * degToPulseConst_1);
      // swerve_deg1 - swerve_off = - deg1* degToPulseConst_1
      // -deg1 = swerve_deg1 - swerve_off / degToPulseConst_1
      

      p = kp * error;
      i += error; 
      i = constrain(i, -2046, 2046);
      d = (error - preverror) * kd;
      preverror = error;
      edit = p + (i * ki) + d;
      if (p_edit1 > 1000)
      {
        edit = 1000;
      }
      else if (p_edit1 < -1000)
      {
        edit = -1000;
      }

      if (swerveNo == 1)
      {
        spin_drive(1, edit);
      }
      else if (swerveNo == 2)
      {
        spin_drive(2, edit);
      }
      else
      {
        spin_drive(3, edit);
      }

      if (abs(error) < marginError)
      {
        if (atTarget == false)
        {
          targetTime = millis();
        }
        atTarget = true;
        if (millis() - targetTime > 500)
        {
          stopAll2();
          stopFlag = false;
          break;
        }
      }
      else
      {
        atTarget = false;
      }
    }
  }
}

void getGraph()
{
  // 1st kit
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(1, 240, p_kp1, p_ki1, p_kd1); //
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(1, 0, p_kp1, p_ki1, p_kd1);
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(1, 240, p_kp1, 0, 0); //
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(1, 0, p_kp1, p_ki1, p_kd1);
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(1, 240, p_kp1, 0, p_kd1); //
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(1, 0, p_kp1, p_ki1, p_kd1);
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  // 2nd kit

  tuneSwerveKit(2, 240, p_kp1, p_ki1, p_kd1); //
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(2, 0, p_kp1, p_ki1, p_kd1);
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(2, 240, p_kp1, 0, 0); //
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(2, 0, p_kp1, p_ki1, p_kd1);
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(2, 240, p_kp1, 0, p_kd1); //
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(2, 0, p_kp1, p_ki1, p_kd1);
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);

  // 3rd kit
  tuneSwerveKit(3, 240, p_kp1, p_ki1, p_kd1); //
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(3, 0, p_kp1, p_ki1, p_kd1);
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(3, 240, p_kp1, 0, 0); //
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(3, 0, p_kp1, p_ki1, p_kd1);
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(3, 240, p_kp1, 0, p_kd1); //
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
  tuneSwerveKit(3, 0, p_kp1, p_ki1, p_kd1);
  while (digitalRead(SW_Bla) == 1)
  {
    stopAll2();
  }
  delay(1000);
}

void printPos()
{
  getRobotPosition();
  float gyro_pos = -readGyro();
  Serial.print(x_glob);
  Serial.print(",");
  Serial.print(y_glob);
  Serial.print(",");
  Serial.println(gyro_pos);
}

void straightCmdExec()
{
  Serial.println("=================================");
  Serial.println();

  Serial.println("Enter distance (m) :");
  straightMove(waitForInput().toFloat());
}

void circularCmdExec()
{
  Serial.println("=================================");
  Serial.println();
  Serial.println("1. Full Circle Locked Heading");
  Serial.println("2. Half Circle Locked Heading");
  Serial.println("3. Full Circle Tangent Heading");
  Serial.println("4. Half Circle Tangent Heading");
  String cmd = waitForInput();
  float rad;
  Serial.println("=================================");
  Serial.println();
  Serial.println("Enter Radius (m) :");
  rad = waitForInput().toFloat(); // will be rounded to 2 decimal degits
  switch (cmd.toInt())
  {
  case 1:
    Serial.println("Course Set for Full Circle Locked Heading");
    fullCircleMove(rad, true);
    break;
  case 2:
    Serial.println("Course Set for Half Circle Locked Heading");
    halfCircleMove(rad, true);
    break;
  case 3:
    Serial.println("Course Set for Full Circle Tangent Heading");
    fullCircleMove(rad, false);
    break;
  case 4:
    Serial.println("Course Set for Half Circle Tangent Heading");
    halfCircleMove(rad, false);
    break;
  default:
    Serial.println("Invalid Input");
    return;
  }
}

void rectangleCmdExec()
{
  float wd, hg;
  Serial.println("=================================");
  Serial.println();
  Serial.println("Enter Width (m) :");
  wd = waitForInput().toFloat(); // will be rounded to 2 decimal degits
  Serial.println("Enter Height (m)");
  hg = waitForInput().toFloat(); // will be rounded to 2 decimal degits
  Serial.println("Rectagle / Square Path Set!");
  rectangleMove(wd, hg);
}

void triangleCmdExec()
{
  Serial.println("=================================");
  Serial.println();
  Serial.println("1. Normal Triangle");
  Serial.println("2. Right Triangle");
  String cmd = waitForInput();
  float bs, hg;
  Serial.println("=================================");
  Serial.println();
  switch (cmd.toInt())
  {
  case 1:
    Serial.println("Enter Base Length (m) :");
    bs = waitForInput().toFloat(); // will be rounded to 2 decimal degits
    Serial.println("Enter Height Length (m)");
    hg = waitForInput().toFloat(); // will be rounded to 2 decimal degits
    Serial.println("Course Set for Normal Triangle");
    triangleMove(bs, hg, false);
    break;
  case 2:
    Serial.println("Enter Base Length (m) :");
    bs = waitForInput().toFloat(); // will be rounded to 2 decimal degits
    Serial.println("Enter Height Length (m)");
    hg = waitForInput().toFloat(); // will be rounded to 2 decimal degits
    Serial.println("Course Set for Right Triangle");
    triangleMove(bs, hg, true);
    break;
  default:
    Serial.println("Invalid Input");
    return;
  }
}

String waitForInput()
{
  int commPos = 0;        // Register used to keep track of the index of the command
  char command[10] = {0}; // Array to store the incoming commands
  String cmd = "";        // merge char cmd
  int index;
  for (index = 0; index < 10; index++) // Initialize the command array to NULL
    command[index] = 0;
  while (true)
  {
    if (Serial.available())
    {
      int commLen = Serial.readBytesUntil('\n', &command[0], 10); // Store the command in an array and store the length of the incoming command
      for (index = 0; index < 10; index++)
      {
        // Echo the incoming command
        Serial.print(command[index]);
        cmd += command[index];
      }

      Serial.println();
      return cmd;
    }
  }
}
