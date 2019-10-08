#include <Adafruit_LSM9DS1.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

// MegaPingTest.ino
// BDK:ESE421:2019C
// Week 2 Lab Sketch -- Test Ping Sensor

Servo steeringServo;
double sum1 = 0.0;
double looptimes =0;
double sum1number = 0.0;
double deltaR = 0.0;
double servoAngleDeg = 0.0;
double LdistanceToObj;
double RdistanceToObj;
double FdistanceToObj;
double rb; // inherent gyro z error
double slope; // slope of surface
float pingDistanceCM = 0.0;
double const velWithPWM100 = 1.40 / 2.95; // car went 1.4 m in 2.95 seconds at PWM 100;
double const LOSTWALL = 40;
double const DISTWALLTHRESH = 30;

// ping sensor on the left
#define LpingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define LpingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define LpingGrndPin 27 // ping sensor ground pin (use digital pin as ground)

// ping sensor on the right UPDATE
#define RpingTrigPin 6 // ping sensor trigger pin (output from Arduino)
#define RpingEchoPin 5 // ping sensor echo pin (input to Arduino)

// ping sensor on the front
#define FpingTrigPin 22 // ping sensor trigger pin (output from Arduino)
#define FpingEchoPin 24 // ping sensor echo pin (input to Arduino)
#define FpingGrndPin 26 // ping sensor ground pin (use digital pin as ground)

#define servoPin 7 // pin for servo signal

// motor pins
#define motorFwdPin 8 // HIGH for FWD; LOW for REV
#define motorRevPin 9 // LOW for FWD; HIGH for REV
#define motorLPWMPin 10 // Left Motor Speed Control
#define motorRPWMPin 11 // Right Motor Speed Control
byte motorLPWM=100;
byte motorRPWM=100;

//// IMU
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

// centering wheel
#define NORMAL 3 // do usual task
#define LEFT 0
#define CENTER 1
#define RIGHT 2
#define PINGDMAX 100
#define PINGDADJUST 50
#define OMEGAX 3

// constant variables

double KI = -0.3;
double KP = -4.5;
double fixedTime = 500;
int state; // (0 = heading hold, 1 = wall following, 2 = drive around box)
double dt;
unsigned long microlast;
double headingEstimate = 0.0;

void setup() {
    state = 1;
   
    Serial.begin(115200);
    // left ping sensor
    pinMode(LpingGrndPin,OUTPUT); digitalWrite(LpingGrndPin,LOW);
    pinMode(LpingTrigPin,OUTPUT);
    pinMode(LpingEchoPin,INPUT);

    // front ping sensor
    pinMode(FpingGrndPin,OUTPUT); digitalWrite(FpingGrndPin,LOW);
    pinMode(FpingTrigPin,OUTPUT);
    pinMode(FpingEchoPin,INPUT);

    // right ping sensor
    pinMode(RpingTrigPin,OUTPUT);
    pinMode(RpingEchoPin,INPUT);
    
    // servo
    steeringServo.attach(servoPin);
    setServoAngle(servoAngleDeg);

    // motors
    pinMode(motorFwdPin,OUTPUT); digitalWrite(motorFwdPin,HIGH);
    pinMode(motorRevPin,OUTPUT); digitalWrite(motorRevPin,LOW);
    pinMode(motorLPWMPin,OUTPUT); analogWrite(motorLPWMPin,motorLPWM);
    pinMode(motorRPWMPin,OUTPUT); analogWrite(motorRPWMPin,motorRPWM);

    // IMU
    lsm.begin();
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);
    // find angle of slope and turn bias
    double sum = 0.0;
    double accelZ = 0.0;
    for (int i = 0; i < 100; i++) {
      sum += g.gyro.z;
      accelZ += a.acceleration.z;
    }
    rb = sum / 100.0;
    slope = acos(accelZ/10.0/9.81) * 360.0 / (2.0 * 3.14159265358979323846) - 11.5;
}

//////////////////////////////////////////////////////////////////
void loop() {
    dt = 0.000001*(micros()-microlast);
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);
    RdistanceToObj = getPingDistanceCM(2);
    if (RdistanceToObj >= LOSTWALL) {
      state = 0;  
    } else if (RdistanceToObj < LOSTWALL) {
        state = 1;
    } 
    // state param: 0 = heading hold, 1 = wall following, 2 = around box
    if (state == 0) {
      double gyroZ = g.gyro.z - rb;
      headingEstimate += -gyroZ*dt*100;
      // Serial.println(headingEstimate);
      servoAngleDeg = constrain(KP * (-headingEstimate), -20, 20);
      // Serial.println(servoAngleDeg);
      // switch to wall following state if wall is detected
    } else if (state == 1) {
      wallFollow(1); 
    } else if (state == 2) {
      driveAroundBox();  
    }
    // Serial.println(servoAngleDeg);
    setServoAngle(servoAngleDeg);
    digitalWrite(motorFwdPin, HIGH);
    digitalWrite(motorRevPin,LOW);
    analogWrite(motorLPWMPin,motorLPWM);
    analogWrite(motorRPWMPin,motorRPWM);
    microlast=micros(); 
}

// @input state: 1 is simpmle wall following, state=2 is drive around box
void wallFollow(int state) {
      // get distance to object with left ping sensor
      LdistanceToObj = getPingDistanceCM(0);
      // Serial.println("wall follow: " ); Serial.print(LdistanceToObj);
      // get distance to object with front ping sensor
      FdistanceToObj = getPingDistanceCM(1);
      // get distance to object with right ping sensor
      RdistanceToObj = getPingDistanceCM(2);
      // Serial.println(RdistanceToObj);
      
      // set speed of motors based on distance to object
      if (LdistanceToObj < 10 || FdistanceToObj < 10 || RdistanceToObj < 10) {
        motorLPWM=0;
        motorRPWM=0;
      } else {
        motorLPWM=100;
        motorRPWM=100;
      }
      
      // set servo degree based on distance to object
      static double deltaI = 0.0;
      double error = DISTWALLTHRESH-RdistanceToObj;
      deltaI +=  error * dt;
      servoAngleDeg = constrain((KP*error+KI*deltaI), -20, 20);
      // Serial.println(servoAngleDeg);
      if (state == 2) {
        motorLPWM=75;
        motorRPWM=75;
        if (LdistanceToObj > DISTWALLTHRESH) {
          driveAroundBox();  
        }  
      }
}

void updaterb() {
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);
    // find angle of slope and turn bias
    double sum = 0.0;
    for (int i = 0; i < 10; i++) {
      sum += g.gyro.z;
    }
    rb = sum / 10.0;
}

void driveAroundBox() {
  double sumLdistanceToObj = 0.0;
  int numReadings = 10;
  for (int i = 0; i < numReadings; i++) {
    sumLdistanceToObj += getPingDistanceCM(0); 
  }
  LdistanceToObj = sumLdistanceToObj / numReadings;
 // Serial.println("drive around box: " ); Serial.print(LdistanceToObj);
  while (LdistanceToObj > 30) {
      setServoAngle(-20);
      double sumLdistanceToObj = 0.0;
      int numReadings = 10;
      for (int i = 0; i < numReadings; i++) {
        sumLdistanceToObj += getPingDistanceCM(0); 
      }
      LdistanceToObj = sumLdistanceToObj / numReadings;
      // Serial.println("update dist: " ); Serial.print(LdistanceToObj);
      // delay(500);
  }
  wallFollow(2);
}
////////////////////////////////////////////////////////////
// Ping Sensor -- update value of pingDistanceCM
////////////////////////////////////////////////////////////
// left ping sensor = 0
// front ping sensor = 1
// right ping sensor = 2
double getPingDistanceCM(int ping) {
  //
  // 3000 us timeout implies maximum distance is 51cm
  // but in practice, actual max larger?
  //
  const long timeout_us = 3000;
  //
  // pingTrigPin = trigger pin
  // pingEchoPin = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  
  if (ping == 0) {
      digitalWrite(LpingTrigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(LpingTrigPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(LpingTrigPin, LOW);
        //
      // The echo pin is used to read the signal from the PING))): a HIGH
      // pulse whose duration is the time (in microseconds) from the sending
      // of the ping to the reception of its echo off of an object.
      //
        unsigned long echo_time;
        echo_time = pulseIn(LpingEchoPin, HIGH, timeout_us);
        if (echo_time == 0)
       {
          echo_time = timeout_us;
       }
       pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);
    } else if (ping == 1) {
        digitalWrite(FpingTrigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(FpingTrigPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(FpingTrigPin, LOW);
        unsigned long echo_time;
        echo_time = pulseIn(FpingEchoPin, HIGH, timeout_us);
        if (echo_time == 0)
       {
          echo_time = timeout_us;
       }
       pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);
   }  else if (ping == 2) {
        digitalWrite(RpingTrigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(RpingTrigPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(RpingTrigPin, LOW);
        unsigned long echo_time;
        echo_time = pulseIn(RpingEchoPin, HIGH, timeout_us);
        if (echo_time == 0)
       {
          echo_time = timeout_us;
       }
       pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);
   }
  
  //
  // return the distance in centimeters
  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  // divide by 2 because we measure "round trip" time for echo
  // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
  // = 0.017*echo_time
  //
  
  return pingDistanceCM;
}


void setServoAngle(double sDeg)
{
//
//  Update ServoCenter_us as Required for installation bias
//  CAREFUL: make small increments (~100) to iterate
//  100us is about 20deg (higher values --> more right steering)
//  wrong ServoCenter values can damage servo
//
    double ServoCenter_us = 1200.0;
    double ServoScale_us = 8.0;    // micro-seconds per degree
//
//  NEVER send a servo command without constraining servo motion!
//  -->large servo excursions could damage hardware or servo<--
//
    double t_us = constrain(ServoCenter_us + ServoScale_us * sDeg, ServoCenter_us-200, ServoCenter_us+200);
    steeringServo.writeMicroseconds(t_us);
}

void travel(double x, double v) {
  double time = x / v;
  double desiredPWM = v * 100 / velWithPWM100;
  motorLPWM=desiredPWM;
  motorRPWM=desiredPWM;
  digitalWrite(motorFwdPin, HIGH);
  digitalWrite(motorRevPin,LOW);
  analogWrite(motorLPWMPin,motorLPWM);
  analogWrite(motorRPWMPin,motorRPWM);   
}
