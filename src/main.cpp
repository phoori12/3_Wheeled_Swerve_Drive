#include <Arduino.h>

//Module 1
#define INA_1 41
#define INB_1 40
#define PWM_1 37
#define prox_1 30
#define encA1 2
#define encB1 3

//Module 2
#define INA_2 39
#define INB_2 38
#define PWM_2 36
#define prox_2 31
#define encA2 4
#define encB2 5

//Module 3
#define INA_3 35
#define INB_3 34
#define PWM_3 33
#define prox_3 32
#define encA3 6
#define encB3 7
#define SW_Red 10
#define SW_Yel 11
#define SW_Bla 12

float degToRad(float val);
float radToDeg(float val);
void stopAll2();
void degAdj1_posCon();
void degAdj2_posCon();
void degAdj3_posCon();
void swerveDrive(float spd, float dir, float omega);
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

volatile long ENC1_Count = 0;
volatile long ENC2_Count = 0;
volatile long ENC3_Count = 0;

// Degree Adjustor Position Control Variables //
float pulsePerDeg = 11.377778;

// PID Variables //
const float p_kp1 = 8.50f, p_ki1 = 0.1f, p_kd1 = 1.00f; // kp 8.00f
const float p_kp2 = 7.30f, p_ki2 = 0.04f, p_kd2 = 1.00f;
const float p_kp3 = 7.50f, p_ki3 = 0.03f, p_kd3 = 1.00f;
float p_edit1, p_error1 = 0, p_preverror1 = 0, p_p1 = 0, p_i1 = 0, p_d1 = 0;
float p_edit2, p_error2 = 0, p_preverror2 = 0, p_p2 = 0, p_i2 = 0, p_d2 = 0;
float p_edit3, p_error3 = 0, p_preverror3 = 0, p_p3 = 0, p_i3 = 0, p_d3 = 0;

volatile int swerve_deg1 = 0;
volatile int swerve_deg2 = 0;
volatile int swerve_deg3 = 0;

volatile bool deg1Flag = false;
volatile bool deg2Flag = false;
volatile bool deg3Flag = false;

// Swerve Offset and Deg Conversion Variables //
int swerve_off1 = 1800;
int swerve_off2 = -1100;
int swerve_off3 = 400;

int swerve_right1 = -300; //-2400 
int swerve_right2 = -3200; // -5300
int swerve_right3 = 2500; // 4600

int marginError = 150;

const float pulseToDegConst = 23.3333;
void setDegSwerve(int deg1, int deg2, int deg3) {
  if (deg1 >= 360) {
    deg1 -= 360;
  } else if (deg1 <= -360) {
    deg1 += 360;
  }
  if (deg2 >= 360) {
    deg2 -= 360;
  } else if (deg2 <= -360) {
    deg2 += 360;
  }
  if (deg3 >= 360) {
    deg3 -= 360;
  } else if (deg3 <= -360) {
    deg3 += 360;
  }

  swerve_deg1 = swerve_off1 - (deg1 * pulseToDegConst);
  swerve_deg2 = swerve_off2 - (deg2 * pulseToDegConst);
  swerve_deg3 = swerve_off3 - (deg3 * pulseToDegConst);
}

IntervalTimer subroutine_posCon1;
IntervalTimer subroutine_posCon2;
IntervalTimer subroutine_posCon3;

uint32_t readCountTime;
uint32_t sendSpeedTime;

union packed_int {
  int32_t i;
  byte b[4];
} m1, m2, m3;

void setup() {
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

  Serial.begin(9600);
  Serial5.begin(115200); //motor 1
  Serial4.begin(115200); //motor 2
  Serial3.begin(115200); //motor 3

  // Homing before start //
  while (digitalRead(SW_Bla) == 1) {
    if (digitalRead(SW_Yel) == 0) {
      spinCCW();
    } else {
      spin_drive(1, 0);
      spin_drive(2, 0);
      spin_drive(3, 0);
    }
    if (digitalRead(SW_Red) == 0) {
      homeTheta();
      break;
    }
  }
  stopAll2();

  // Start Sub routine (Swerve Dish Position Control) //
  subroutine_posCon1.begin(degAdj1_posCon, 1000);
  subroutine_posCon2.begin(degAdj2_posCon, 1000);
  subroutine_posCon3.begin(degAdj3_posCon, 1000);

  swerve_deg1 = swerve_off1;
  swerve_deg2 = swerve_off2;
  swerve_deg3 = swerve_off3;
  delay(1000);
  setDegSwerve(0, 0, 0);
  delay(3000);
}

long lasttime1 = 0, lasttime2 = 0, lasttime3 = 0;
long looptime = 1;

long lasttime_shit = 0;

void loop() {
     
  lasttime_shit = millis();
  while (millis() - lasttime_shit < 5000) {
    swerveDrive(50, 0, 0);
  }
  stopAll2();
  delay(1000);
  lasttime_shit = millis();
  while (millis() - lasttime_shit < 5000) {
    swerveDrive(50, 90, 0);
  }
  stopAll2();
  delay(1000);
  lasttime_shit = millis();
  while (millis() - lasttime_shit < 5000) {
    swerveDrive(50, 180, 0);
  }
  stopAll2();
  delay(1000);
  lasttime_shit = millis();
  while (millis() - lasttime_shit < 5000) {
    swerveDrive(50, 270, 0);
  }
  stopAll2();
  delay(1000);
  // lasttime_shit = millis();
  // while (millis() - lasttime_shit < 1000) {
  //   swerve_deg1 = -500;
  //   swerve_deg2 = -500;
  //   swerve_deg3 = -500;
  //   if (millis() - sendSpeedTime > 10)
  //   {
  //     sendSpeedTime = millis();
  //     sendCmd(30, 30, 30);
  //   }
  // }
  // stopAll2();
}

void swerveDrive(float spd, float dir, float omega) {
  float vx,vy;
  float vx1, vy1, vx2, vy2, vx3, vy3;
  float vw1, vw2, vw3, thet1, thet2, thet3;
  vx = spd * cos(degToRad(dir));
  //vx = spd * cosf(dir * DEG_TO_RAD)
  vy = spd * sin(degToRad(dir));
  vx1 = omega + vx;
  vy1 = vy;
  vx2 = -omega*sin(degToRad(30)) + vx;
  vy2 = -omega*sin(degToRad(60)) + vy;
  vx3 = -omega*sin(degToRad(30)) + vx;
  vy3 = omega*sin(degToRad(60)) + vy;

  // TODO Contrain shit
  vw1 = sqrt(pow(vx1, 2) + pow(vy1, 2));
  vw2 = sqrt(pow(vx2, 2) + pow(vy2, 2));
  vw3 = -sqrt(pow(vx3, 2) + pow(vy3, 2));
  if (vw1 > spd) {
    vw1 = spd;
  } else if (vw1 < -spd) {
    vw1 = -spd;
  }

  if (vw2 > spd) {
    vw2 = spd;
  } else if (vw2 < -spd) {
    vw2 = -spd;
  }

  if (vw3 > spd) {
    vw3 = spd;
  } else if (vw3 < -spd) {
    vw3 = -spd;
  }

  if ((millis() - sendSpeedTime > 10) && (deg1Flag && deg2Flag && deg3Flag))
  {
    sendSpeedTime = millis();
    sendCmd(vw1, vw2, vw3);
  } else if (millis() - sendSpeedTime > 10) {
    sendSpeedTime = millis();
    sendCmd(0, 0, 0);
  }

  thet1 = radToDeg(atan2(vx1, vy1));
  thet2 = radToDeg(atan2(vx2, vy2));
  thet3 = radToDeg(atan2(vx3, vy3));

  Serial.print(deg1Flag);
  Serial.print(" ");
  Serial.print(deg2Flag);
  Serial.print(" ");
  Serial.println(deg3Flag);

  setDegSwerve(thet1, thet2, thet3);
}

float degToRad(float val)
{
  // Serial.print(val);
  // Serial.print(" degRad ");
  // Serial.println(val * (PI / 180));
  return val * DEG_TO_RAD;
}

float radToDeg(float val)
{
  // Serial.print(val);
  // Serial.print(" radDeg ");
  // Serial.println(val * (180 / PI));
  return val * RAD_TO_DEG;
}

void stopAll2() {
  spin_drive(1, 0);
  spin_drive(2, 0);
  spin_drive(3, 0);
}

void degAdj1_posCon() {
  if (millis() - lasttime1 >= looptime) {
    lasttime1 = millis();
    if (abs(ENC1_Count - swerve_deg1) > 2)
    {
      p_error1 = swerve_deg1 - ENC1_Count;
      deg1Flag = false;
    }
    else
    {
      p_error1 = 0;
    }

    if (abs(ENC1_Count - swerve_deg1) < marginError)
    {
      deg1Flag = true;
    }
    p_p1 = p_kp1 * p_error1;
    p_i1 += p_error1;
    p_i1 = constrain(p_i1, -2046, 2046);
    p_d1 = (p_error1 - p_preverror1) * p_kd1;
    p_preverror1 = p_error1;
    p_edit1 = p_p1 + (p_i1 * p_ki1) + p_d1;
    if (p_edit1 > 1000) {
      p_edit1 = 1000;
    } else if (p_edit1 < -1000) {
      p_edit1 = -1000;
    }
    spin_drive(1, p_edit1);
  }
}

void degAdj2_posCon() {
  if (millis() - lasttime2 >= looptime) {
    lasttime2 = millis();
    if (abs(ENC2_Count - swerve_deg2) > 2)
    {
      p_error2 = swerve_deg2 - ENC2_Count;
      deg2Flag = false;
    }
    else
    {
      p_error2 = 0;
    }

    if (abs(ENC2_Count - swerve_deg2) < marginError)
    {
      deg2Flag = true;
    }

    p_p2 = p_kp2 * p_error2;
    p_i2 += p_error2;
    p_i2 = constrain(p_i2, -2046, 2046);
    p_d2 = (p_error2 - p_preverror2) * p_kd2;
    p_preverror2 = p_error2;
    p_edit2 = p_p2 + (p_i2 * p_ki2) + p_d2;
    if (p_edit2 > 1000) {
      p_edit2 = 1000;
    } else if (p_edit2 < -1000) {
      p_edit2 = -1000;
    }
    // Serial.print(0);
    // Serial.print(" ");
    // Serial.println(p_error2);
    spin_drive(2, p_edit2);
  }
}

void degAdj3_posCon() {
  if (millis() - lasttime3 >= looptime) {
    lasttime3 = millis();
    if (abs(ENC3_Count - swerve_deg3) > 2)
    {
      p_error3 = swerve_deg3 - ENC3_Count;
      deg3Flag = false;
    }
    else
    {
      p_error3 = 0;
    }

    if (abs(ENC3_Count - swerve_deg3) < marginError)
    {
      deg3Flag = true;
    }

    p_p3 = p_kp3 * p_error3;
    p_i3 += p_error3;
    p_i3 = constrain(p_i3, -2046, 2046);
    p_d3 = (p_error3 - p_preverror3) * p_kd3;
    p_preverror3 = p_error3;
    p_edit3 = p_p3 + (p_i3 * p_ki3) + p_d3;
    if (p_edit3 > 1000) {
      p_edit3 = 1000;
    } else if (p_edit3 < -1000) {
      p_edit3 = -1000;
    }
    spin_drive(3, p_edit3);
  }
}

void spinCCW() {
  spin_drive(1, -400);
  spin_drive(2, -400);
  spin_drive(3, -400);
}

void homeTheta() {
  while (digitalRead(prox_1) == 0) {
    spin_drive(1, 650);
  }
  spin_drive(1, -200);
  delay(500);
  while (digitalRead(prox_1) == 0) {
    spin_drive(1, 200);
  }
  spin_drive(1, 0);
  /////////////////////
  while (digitalRead(prox_2) == 0) {
    spin_drive(2, 650);
  }
  spin_drive(2, -200);
  delay(500);
  while (digitalRead(prox_2) == 0) {
    spin_drive(2, 200);
  }
  spin_drive(2, 0);
  ////////////////////
  while (digitalRead(prox_3) == 0) {
    spin_drive(3, 650);
  }
  spin_drive(3, -200);
  delay(500);
  while (digitalRead(prox_3) == 0) {
    spin_drive(3, 200);
  }
  spin_drive(3, 0);
  delay(500);
  ENC1_Count = 0;
  ENC2_Count = 0;
  ENC3_Count = 0;
  delay(100);
}

void stopAll() {
  while (1) {
    spin_drive(1, 0);
    spin_drive(2, 0);
    spin_drive(3, 0);
  }
}

void spin_drive(int mtr, int spd) {
  if (mtr == 1) {
    if (spd > 0) { // CW
      digitalWrite(INA_1, 0);
      digitalWrite(INB_1, 1);
      analogWrite(PWM_1, spd);
    } else if (spd < 0) { // CCW
      digitalWrite(INA_1, 1);
      digitalWrite(INB_1, 0);
      analogWrite(PWM_1, -spd);
    } else {
      digitalWrite(INA_1, 1);
      digitalWrite(INB_1, 1);
      analogWrite(PWM_1, 0);
    }
  } else if (mtr == 2) {
    if (spd > 0) { // CW
      digitalWrite(INA_2, 0);
      digitalWrite(INB_2, 1);
      analogWrite(PWM_2, spd);
    } else if (spd < 0) { // CCW
      digitalWrite(INA_2, 1);
      digitalWrite(INB_2, 0);
      analogWrite(PWM_2, -spd);
    } else {
      digitalWrite(INA_2, 1);
      digitalWrite(INB_2, 1);
      analogWrite(PWM_2, 0);
    }
  } else if (mtr == 3) {
    if (spd > 0) {
      digitalWrite(INA_3, 0);
      digitalWrite(INB_3, 1);
      analogWrite(PWM_3, spd);
    } else if (spd < 0) {
      digitalWrite(INA_3, 1);
      digitalWrite(INB_3, 0);
      analogWrite(PWM_3, -spd);
    } else {
      digitalWrite(INA_3, 1);
      digitalWrite(INB_3, 1);
      analogWrite(PWM_3, 0);
    }
  } else {
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

void sendCmd(float spd1, float spd2, float spd3) {
  m1.i = spd1 * 100;
  m2.i = spd2 * 100;
  m3.i = spd3 * 100;
  const char cmd1[8] = {'#', 's', m1.b[3], m1.b[2], m1.b[1], m1.b[0], '\r', '\n'};
  const char cmd2[8] = {'#', 's', m2.b[3], m2.b[2], m2.b[1], m2.b[0], '\r', '\n'};
  const char cmd3[8] = {'#', 's', m3.b[3], m3.b[2], m3.b[1], m3.b[0], '\r', '\n'};
  for (uint8_t i = 0; i < 8; i++) {
    Serial5.write(cmd1[i]);
    Serial4.write(cmd2[i]);
    Serial3.write(cmd3[i]);
  }

}