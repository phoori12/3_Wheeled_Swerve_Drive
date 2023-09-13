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

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}
MPU6050 mpu;
float gyro_accept = 3.00f;
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
void moveWithDelay(float spd, float dir, float omega, int duration);
void getRobotPosition();
void p2ptrack(float set_x, float set_y, float set_head, bool viaMode = false);
float closestAngle(float a, float b);

volatile long ENC1_Count = 0;
volatile long ENC2_Count = 0;
volatile long ENC3_Count = 0;

// Degree Adjustor Position Control Variables //
float pulsePerDeg = 11.377778;

// PID Variables //
const float p_kp1 = 8.50f, p_ki1 = 0.1f, p_kd1 = 1.00f; // kp 8.00f
const float p_kp2 = 7.30f, p_ki2 = 0.1f, p_kd2 = 1.00f;
const float p_kp3 = 7.50f, p_ki3 = 0.03f, p_kd3 = 1.00f;

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
int swerve_off1 = 1800;
int swerve_off2 = -1100;
int swerve_off3 = 400;

int swerve_right1 = -300;  //-2400
int swerve_right2 = -3200; // -5300
int swerve_right3 = 2500;  // 4600
volatile float prev_deg1, prev_deg2, prev_deg3;

int marginError = 10;

const float DegToPulseConst = 23.3333;
IntervalTimer subroutine_posCon1;
IntervalTimer subroutine_posCon2;
IntervalTimer subroutine_posCon3;

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
const float h_kp = 3.00f, h_ki = 0.00f, h_kd = 1.00f;
float dx, dy, dsm, s_error, d_s, s_edit, compensateTht;
float h_edit, h_error = 0, h_preverror = 0, h_p = 0, h_i = 0, h_d = 0;
long p2pTargetTime = 0;
////////////////////////////////////////////////////////////

#define MAX_SPD 20

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
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(9600);
  Serial5.begin(115200); // motor 1
  Serial4.begin(115200); // motor 2
  Serial3.begin(115200); // motor 3

  // initialize device
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(13, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

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
  Serial.println("First wheel setup");
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
  Serial.println("Second wheel setup");
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
  Serial.println("Third wheel setup");
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
  Serial.println("GO HOME");
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

  // swerve_deg1 = swerve_off1;
  // swerve_deg2 = swerve_off2;
  // swerve_deg3 = swerve_off3;
  // delay(1000);
  setDegSwerve(90, 90, 90, 0, 0, 0);
  delay(3000);

  Serial.println("press red");
  while (digitalRead(SW_Red) == 1)
    ;
  delay(1000);

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(29);
  mpu.setYGyroOffset(-25);
  mpu.setZGyroOffset(58);

  mpu.setXAccelOffset(-5210);
  mpu.setYAccelOffset(-54);
  mpu.setZAccelOffset(1738);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(13));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(13), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  while (digitalRead(SW_Bla) == 1)
  {
    gyro_offset = readGyro();
    Serial.println(gyro_offset);
  }
}

long lasttime1 = 0, lasttime2 = 0, lasttime3 = 0;
uint32_t looptime = 1;

long lasttime_shit = 0;

uint32_t localizeTime = 0;
void loop()
{
  // headingControl(30, -90, 0);
  //  setDegSwerve(90,90,90,0 ,0 ,0);
  //  while (digitalRead(SW_Red) == 1)
  //    ;
  //  delay(1000);
  //  setDegSwerve(0,0,0,0 ,0 ,0);
  //  while (digitalRead(SW_Red) == 1)
  //    ;
  //  delay(1000);
  //  setDegSwerve(270,270,270,0 ,0 ,0);
  //  while (digitalRead(SW_Red) == 1)
  //    ;
  //  delay(1000);
  //  setDegSwerve(180,180,180,0 ,0 ,0);
  //  while (digitalRead(SW_Red) == 1)
  //    ;
  //  delay(1000);
  //  setDegSwerve(200,200,200,0 ,0 ,0);
  //  while (digitalRead(SW_Red) == 1)
  //    ;
  //  delay(1000);
  p2ptrack(0, 0.5, -90);
  stopAll2();
  delay(1000);
  p2ptrack(0.5, 0.5, -180);
  stopAll2();
  delay(1000);
  p2ptrack(0.5, 0, 90); 
  stopAll2();
  delay(1000);
  p2ptrack(0, 0, 0);
  stopAll2();
  delay(1000);
  while (1)
  {
    stopFlag = true;
    stopAll2();
  }
  // stopFlag = true;
  // stopAll2();
  // if (millis() - localizeTime > 10)
  // {
  //   localizeTime = millis();
  //   sendCmd(10, 10, -10);
  //   getRobotPosition();
  // }

  // lasttime_shit = millis();
  // while (millis() - lasttime_shit < 1000)
  // {
  //   if (millis() - sendSpeedTime > 10)
  //   {
  //     sendSpeedTime = millis();
  //     sendCmd(10, 0, 0);
  //     Serial.println(x);
  //   }
  // }
  // lasttime_shit = millis();
  // while (millis() - lasttime_shit < 1000)
  // {
  //   if (millis() - sendSpeedTime > 10)
  //   {
  //     sendSpeedTime = millis();
  //     sendCmd(-10, 0, 0);
  //     Serial.println(x);
  //   }
  // }
}
void p2ptrack(float set_x, float set_y, float set_head, bool viaMode = false)
{
  static volatile float s_prev_error = 0.0f;
  static bool onPoint = false;
  static float prev_compenstateTht = 0.0f;
  static float theta = 0;
  set_head = -set_head;
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

    float gyro_pos = readGyro() - gyro_offset;
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
    // h_edit = (h_error * h_kp) + (h_i * h_ki) + (h_kd * h_d);
    if (set_head >= 0) {
      compensateTht = theta + mapgyro;
    } else {
      compensateTht = theta - mapgyro;
    }
    

    if ((abs(dx) <= 0.05 && abs(dy) <= 0.05) && abs(h_error) <= gyro_accept + gyro_localizeMargin)
    {

      if (onPoint == false)
      {
        p2pTargetTime = millis();
      }
      onPoint = true;
      if (millis() - p2pTargetTime > 500 || viaMode)
      {
        // atTarget = true;
        sendCmd(0, 0, 0);
        Serial.println("Break");
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
    Serial.print(theta);
    Serial.print("\t");
    Serial.print(s_edit);
    Serial.print("\t");
    Serial.print(compensateTht);
    Serial.print("\t");
    Serial.print(-h_edit);
    Serial.print("\t");
    Serial.print(x_glob);
    Serial.print("\t");
    Serial.println(y_glob);
    swerveDrive(s_edit, compensateTht, -h_edit);
    prev_compenstateTht = compensateTht;
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
  // get each Swerve's degree
  // actual_deg1 = (swerve_off1 - swerve_deg1) / DegToPulseConst;
  // actual_deg2 = (swerve_off2 - swerve_deg2) / DegToPulseConst;
  // actual_deg3 = (swerve_off3 - swerve_deg3) / DegToPulseConst;

  //
  // Serial.print(od1);
  // Serial.print("\t");
  // Serial.print(od2);
  // Serial.print("\t");
  // Serial.print(od3);
  // Serial.print(" || ");
  // Serial.print(actual_deg1);
  // Serial.print("\t");
  // Serial.print(actual_deg2);
  // Serial.print("\t");
  // Serial.println(actual_deg3);
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
  float gyro_pos = -(readGyro() - gyro_offset);
  x_glob += (x_frame * cos(degToRad(gyro_pos)) - y_frame * sin(degToRad(gyro_pos)));
  y_glob += (x_frame * sin(degToRad(gyro_pos)) + y_frame * cos(degToRad(gyro_pos)));

  // Serial.print(gyro_pos);
  // Serial.print("\t");
  // Serial.print(x_glob);
  // Serial.print("\t");
  // Serial.println(y_glob);
  ///////////////////////////////
}

void moveWithDelay(float spd, float dir, float head, int duration)
{
  // TODO
}

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
    vx1 = omega + vx;
    vy1 = vy;
    vx2 = -omega * sin(degToRad(30)) + vx;
    vy2 = -omega * sin(degToRad(60)) + vy;
    vx3 = -omega * sin(degToRad(30)) + vx;
    vy3 = omega * sin(degToRad(60)) + vy;

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
      thet1 = 0;
    }
    else
    {
      thet1 = radToDeg(atan2(vy1, vx1));
    }

    if (vx2 == 0 && vy2 == 0)
    {
      thet2 = 0;
    }
    else
    {
      thet2 = radToDeg(atan2(vy2, vx2));
    }

    if (vx3 == 0 && vy3 == 0)
    {
      thet3 = 0;
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
      // priorityDegPosCon(thet1, thet2, thet3);
    }
    else
    {
      setDegSwerve(thet1, thet2, thet3, vw1, vw2, vw3);
    }
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

      if (abs(ENC1_Count - swerve_deg1) > 2)
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

      if (abs(ENC2_Count - swerve_deg2) > 2)
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

      if (abs(ENC3_Count - swerve_deg3) > 2)
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
    // Serial.println("still here");
  }
  Serial.println("done");
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
    if (abs(ENC1_Count - swerve_deg1) > 2)
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
    if (abs(ENC2_Count - swerve_deg2) > 2)
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
    if (abs(ENC3_Count - swerve_deg3) > 2)
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

  if (abs(dir) > 180.0f)
  {
    int signum = 0;
    if (dir > 0)
      signum = 1;
    if (dir < 0)
      signum = -1;
    if (dir == 0)
      signum = 0;

    dir = -(signum * 360) + dir;
  }

  return dir;
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
  // deg1 = prev_deg1 + checkdeg1;
  // deg2 = prev_deg2 + checkdeg2;
  // deg3 = prev_deg3 + checkdeg3;

  if (deg1 >= 360)
  {
    deg1 -= 360;
  }
  else if (deg1 <= -360)
  {
    deg1 += 360;
  }

  if (deg2 >= 360)
  {
    deg2 -= 360;
  }
  else if (deg2 <= -360)
  {
    deg2 += 360;
  }

  if (deg3 >= 360)
  {
    deg3 -= 360;
  }
  else if (deg3 <= -360)
  {
    deg3 += 360;
  }

  actual_deg1 = deg1;
  actual_deg2 = deg2;
  actual_deg3 = deg3;

  swerve_deg1 = swerve_off1 - (deg1 * DegToPulseConst);
  swerve_deg2 = swerve_off2 - (deg2 * DegToPulseConst);
  swerve_deg3 = swerve_off3 - (deg3 * DegToPulseConst);

  float gradUnterscheid_1 = abs(abs(prev_swerve_deg1) - abs(swerve_deg1));
  float gradUnterscheid_2 = abs(abs(prev_swerve_deg2) - abs(swerve_deg2));
  float gradUnterscheid_3 = abs(abs(prev_swerve_deg3) - abs(swerve_deg3));

  // Serial.print(checkdeg1);
  // Serial.print("\t");
  // Serial.print(checkdeg2);
  // Serial.print("\t");
  // Serial.print(checkdeg3);
  // Serial.print("\t");

  if (gradUnterscheid_1 > 1050 || gradUnterscheid_2 > 1050 || gradUnterscheid_3 > 1050)
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

  prev_swerve_deg1 = swerve_deg1;
  prev_swerve_deg2 = swerve_deg2;
  prev_swerve_deg3 = swerve_deg3;
  prev_deg1 = deg1;
  prev_deg2 = deg2;
  prev_deg3 = deg3;
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
  spin_drive(2, -200);
  delay(500);
  while (digitalRead(prox_2) == 0)
  {
    spin_drive(2, 200);
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
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // Serial.print("ypr\t");
    return ypr[0] * 180 / M_PI;
    // Serial.print(ypr[0] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.print(ypr[1] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.print(ypr[2] * 180 / M_PI);
    /*
      mpu.dmpGetAccel(&aa, fifoBuffer);
      Serial.print("\tRaw Accl XYZ\t");
      Serial.print(aa.x);
      Serial.print("\t");
      Serial.print(aa.y);
      Serial.print("\t");
      Serial.print(aa.z);
      mpu.dmpGetGyro(&gy, fifoBuffer);
      Serial.print("\tRaw Gyro XYZ\t");
      Serial.print(gy.x);
      Serial.print("\t");
      Serial.print(gy.y);
      Serial.print("\t");
      Serial.print(gy.z);
    */
    // Serial.println();
  }
}

void headingControl(float spd, float course, float set_head)
{
  float gyro_pos = readGyro() - gyro_offset;
  if (abs(gyro_pos) - set_head > gyro_accept)
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
  // Serial.println(h_edit);
  swerveDrive(spd, course, -h_edit, 0);
}