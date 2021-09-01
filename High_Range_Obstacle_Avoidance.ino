// Sending status to other robot ******* (sending serial data to esp32)
// This robot will not recieve anything from serial
#include <TaskScheduler.h>
int inputValue = 0;                                         //ESP SERIAL
String inputString = "";
boolean stringComplete = false;                             //
int SIMBotNumber = 1;
String readings;                                            //
//------------- SIMBOT ------------------------
// PWM Pins for
int R_PWM = 10;
int L_PWM = 9;
//  Motor Direction Pins
int R_direction = 5;
int L_direction = 6;
#define Forward 1
#define Reverse 0
//  Motor Encoder Pins
int R_encoder = 2;
int L_encoder = 3;
long int encoderLPos = 0;
long int encoderRPos = 0;
int IR_enable = 4;
int OT = 900; // 0 white close obstacle -- 1023 no obstacle
// Variables for 5 IR proximity sensors
int OR, ORF, OF, OLF, OL;
float L2, L1, F0, R1, R2;
bool front_obstacle = 0;
float F, Re, R, L = 0;
float  dia = 0.034  ;         // wheel diameter (in mm)
float Dl, Dr, Dc, Ori_ch;
float Ar, Al, A;
float ER = 1400;      //1m = 1450 rev
float x = 0;           // x initial coordinate of mobile robot
float y = 0;           // y initial coordinate of mobile robot
float Ori  = 0;       // The initial orientation of mobile robot
float Pi = 3.14;
float b = 0.084 ;     // b is the wheelbase of the mobile robot in mm
float grid = 170;
#include <MatrixMath.h>
#define N  (32)
#define M  (5)
mtx_type cases[N][M]; 
bool debug = 0;
float xp =0;
float yp =0;
float xc =0;
float yc =0;
float grid_x=0;
float grid_y=0;
  float xStep =0;
  float yStep = 0;
// IMU Headers
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw;
bool R_en, L_en;
float D = 0;
float DD = 0;
// SERIAL COMMUNICATION                         //ESP SERIAL
void t1Callback();
Task t1(250, TASK_FOREVER, &t1Callback);
Scheduler runner;                               //
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
bool show = 0;
void setup() {
  Serial.begin (57600);                             //ESP SERIAL
  // Taking Inpute from Encoders
  pinMode(R_encoder, INPUT);
  pinMode(L_encoder, INPUT);
  pinMode(11, OUTPUT);
  pinMode(IR_enable, OUTPUT);
  // Counting revolutions of right and left wheel
  attachInterrupt(digitalPinToInterrupt(R_encoder), do_REncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_encoder), do_LEncoder, CHANGE);
  create_combinations();
  SetIMU();
  runner.addTask(t1);                               //ESP SERIAL
  t1.enable();                                      //
}
bool pos = 1;
void loop() {
  runner.execute();
  yaw = current_yaw();
  IR_proximity_read();
  check_obstacle();
  avoid_and_move();
}
void create_combinations() {
  for (int j = 0; j < M; j++)
  {
    for (int i = 0; i < N; i++)
    {
      if (j == 0)
      {
        if (i < 16)
        {
          cases[i][j] = 0;
        }
        else
        {
          cases[i][j] = 1;
        }
      }
      if (j == 1)
      {
        if (i < 8)
        {
          cases[i][j] = 0;
        }
        else if (i >= 8 && i < 16)
        {
          cases[i][j] = 1;
        }
        else if (i >= 16 && i < 24)
        {
          cases[i][j] = 0;
        }
        else if (i >= 24 && i < 32)
        {
          cases[i][j] = 1;
        }
      }
      if (j == 2)
      {
        if (i < 4)
        {
          cases[i][j] = 0;
        }
        else if (i >= 4 && i < 8)
        {
          cases[i][j] = 1;
        }
        else if (i >= 8 && i < 12)
        {
          cases[i][j] = 0;
        }
        else if (i >= 12 && i < 16)
        {
          cases[i][j] = 1;
        }
        else if (i >= 16 && i < 20)
        {
          cases[i][j] = 0;
        }
        else if (i >= 20 && i < 24)
        {
          cases[i][j] = 1;
        }
        else if (i >= 24 && i < 28)
        {
          cases[i][j] = 0;
        }
        else if (i >= 28 && i < 32)
        {
          cases[i][j] = 1;
        }
      }
      if (j == 3)
      {
        if (i < 2)
        {
          cases[i][j] = 0;
        }
        else if (i >= 2 && i < 4)
        {
          cases[i][j] = 1;
        }
        else if (i >= 4 && i < 6)
        {
          cases[i][j] = 0;
        }
        else if (i >= 6 && i < 8)
        {
          cases[i][j] = 1;
        }
        else if (i >= 8 && i < 10)
        {
          cases[i][j] = 0;
        }
        else if (i >= 10 && i < 12)
        {
          cases[i][j] = 1;
        }
        else if (i >= 12 && i < 14)
        {
          cases[i][j] = 0;
        }
        else if (i >= 14 && i < 16)
        {
          cases[i][j] = 1;
        }
        else if (i >= 16 && i < 18)
        {
          cases[i][j] = 0;
        }
        else if (i >= 18 && i < 20)
        {
          cases[i][j] = 1;
        }
        else if (i >= 20 && i < 22)
        {
          cases[i][j] = 0;
        }
        else if (i >= 22 && i < 24)
        {
          cases[i][j] = 1;
        }
        else if (i >= 24 && i < 26)
        {
          cases[i][j] = 0;
        }
        else if (i >= 26 && i < 28)
        {
          cases[i][j] = 1;
        }
        else if (i >= 28 && i < 30)
        {
          cases[i][j] = 0;
        }
        else if (i >= 30 && i < 32)
        {
          cases[i][j] = 1;
        }
      }
      if (j == 4)
      {
        if (i % 2 == 0)
        {
          cases[i][j] = 0;
        }
        else
        {
          cases[i][j] = 1;
        }
      }
    }
  }
  //Matrix.Print((mtx_type*)cases, N, M, "cases");
}
void check_obstacle() {
  if (OL < OT) {
    L2 = 1;
  }
  else {
    L2 = 0;
  }
  if (OLF < OT) {
    L1 = 1;
  }
  else {
    L1 = 0;
  }
  if (OF < OT) {
    F0 = 1;
  }
  else {
    F0 = 0;
  }
  if (ORF < OT) {
    R1 = 1;
  }
  else {
    R1 = 0;
  }
  if (ORF < OT) {
    R1 = 1;
  }
  else {
    R1 = 0;
  }
  if (OR < OT) {
    R2 = 1;
  }
  else {
    R2 = 0;
  }
}
void Go() {
  F = 1;
  Re = 0;
  R = 0;
  L = 0;
  analogWrite(R_PWM, 100);
  digitalWrite(R_direction, Forward);
  analogWrite(L_PWM, 104);
  digitalWrite(L_direction, Forward);
  pos_estimate();
  print_pos(show);
}
void retreat() {
  F = 0;
  Re = 1;
  R = 0;
  L = 0;
  analogWrite(R_PWM, 40);
  digitalWrite(R_direction, Reverse);
  analogWrite(L_PWM, 40);
  digitalWrite(L_direction, Reverse);
  pos_estimate();
  print_pos(show);
}
void right() {
  F = 0;
  Re = 0;
  R = 1;
  L = 0;
  pos_estimate();
  analogWrite(R_PWM, 40);
  digitalWrite(R_direction, Reverse);
  analogWrite(L_PWM, 180);
  digitalWrite(L_direction, Forward);
  print_pos(show);
}
void left() {
  F = 0;
  Re = 0;
  R = 0;
  L = 1;
  pos_estimate();
  analogWrite(R_PWM, 180);
  digitalWrite(R_direction, Forward);
  analogWrite(L_PWM, 40);
  digitalWrite(L_direction, Reverse);
  print_pos(show);
}
void Stop() {
  analogWrite(R_PWM, 255);
  analogWrite(L_PWM, 255);
}
void pos_estimate() {
  Dl = Pi * dia * (encoderLPos / (ER));
  Dr = Pi * dia * (encoderRPos / (ER));
  Dc = (( Dl + Dr) / 2) ;
  A = -yaw;
 float Step = abs(Dc - D);
  if ((Step) != 0)
  {
    xc = Step * cos (A * (3.14 / 180));
xStep = abs(xc - xp);
 yStep = abs(yc - yp);
  if (xStep != 0)
  {
    if (A<90 && A>-90)
    x = x + xStep;
    else
    x = x - xStep;
    xp = xc;
    grid_x=grid*x;
  }
  if (yStep != 0)
  {
    if (A<180 && A>0)
    y = y + yStep;
    else
    y = y - yStep;
    yp = yc;
    grid_y=grid*y;
  }
  D=Dc;
  }
}
void print_pos(bool P)
{
  if (P == 1) {
    Serial.print(x);
    Serial.print(',');
    Serial.print(y);
    Serial.print(',');
    Serial.println(-yaw);
  }
}
void IR_proximity_read() {   // read IR sensors
  int n = 5; // average parameter
  digitalWrite(IR_enable, HIGH);  //IR Enable
  OR = 0;
  ORF = 0;
  OF = 0;
  OLF = 0;
  OL = 0;
  for (int i = 0; i < n; i++) {
    OL += analogRead(A3);
    OLF += analogRead(A2);
    OF += analogRead(A1);
    ORF += analogRead(A0);
    OR += analogRead(A7);
    delay(5);
  }
  OR /= n;
  ORF /= n;
  OF /= n;
  OLF /= n;
  OL /= n;
}
void Send_sensor_readings() {
  Serial.print(L2);
  Serial.print(' ');
  Serial.print(L1);
  Serial.print(' ');
  Serial.print(F0);
  Serial.print(' ');
  Serial.print(R1);
  Serial.print(' ');
  Serial.println(R2);
}
void avoid_and_move() {
  float l2, l1, f0, r1, r2;
  int caseNo;
  for (int i = 0; i < N; i++)
  {
    l2 = cases[i][0];
    l1 = cases[i][1];
    f0 = cases[i][2];
    r1 = cases[i][3];
    r2 = cases[i][4];
    if (L2 == l2 && L1 == l1 && F0 == f0 && R1 == r1 && R2 == r2)
    {
      caseNo = i;
    }
  }
  if (caseNo == 0)
  {
    Go();
  }
  else if (caseNo == 1 || caseNo == 2 || caseNo == 3 || caseNo == 5 || caseNo == 6 || caseNo == 7 || caseNo == 9 ||  caseNo == 11 || caseNo == 13 || caseNo == 15)
  {
    left();
  }
  else if (caseNo == 8 || caseNo == 12 || caseNo == 16 || caseNo == 18 || caseNo == 20 || caseNo == 22 || caseNo == 24 ||  caseNo == 26 || caseNo == 28 || caseNo == 30)
  {
    right();
  }
  else if (caseNo == 4 || caseNo == 10 || caseNo == 14 || caseNo == 17 || caseNo == 19 || caseNo == 21 || caseNo == 23 || caseNo == 25 || caseNo == 27 || caseNo == 29 || caseNo == 31)
  {
    Stop();
  }
  else
  {
    Stop();
  }
}
// Simbot Functions
void do_LEncoder() {
  if (F == 1)
  {
    encoderLPos++;
  }
  else if (Re == 1)
  {
    encoderLPos--;
  }
  else if (R == 1)
  {
    encoderLPos++;
  }
  else if (L == 1)
  {
    encoderLPos--;
  }
}
void do_REncoder() {
  if (F == 1)
  {
    encoderRPos++;
  }
  else if (Re == 1)
  {
    encoderRPos--;
  }
  else if (R == 1)
  {
    encoderRPos--;
  }
  else if (L == 1)
  {
    encoderRPos++;
  }
}
void SetEncoder() {
  encoderLPos = 0;
  encoderRPos = 0;
}
void SetIMU() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.
  // initialize device
  if (debug)
    Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  // verify connection
  if (debug)
    Serial.println(F("Testing device connections..."));
  if (debug)
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // wait for ready
  //    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //    while (Serial.available() && Serial.read()); // empty buffer
  //    while (!Serial.available());                 // wait for data
  //    while (Serial.available() && Serial.read()); // empty buffer again
  // load and configure the DMP
  if (debug)
    Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    if (debug)
      Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    if (debug)
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    if (debug) {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
  }
}
float current_yaw() {
  float yaw;
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[0] * 180 / M_PI;
  }
  return yaw;
}
// ================================================================
// ===               SERIAL TASK CODE                           ===
// ================================================================
void t1Callback() {
  //  char msg[30];
  //  sprintf(msg,"%d,%d,%d",x*100,y*100,yaw*(-100));
  //return readings;
  Serial.print("S1,");
  Serial.print(grid_x); Serial.print(',');
  Serial.print(grid_y); Serial.print(',');
  Serial.println(A);
}
