#include <Adafruit_LSM9DS1.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

// defining global variables and constants
double dt;
unsigned long microlast;
double const velWithPWM100 = 1.40 / 2.95; // car went 1.4 m in 2.95 seconds at PWM 100;
double KI = 0;
double KP = -1.3;
double currAngle = 0.0;
double rb; // inherent gyro z error
double desiredAngle = 0.0;
double wheelTurnBias;
double LdistanceToObj;
double RdistanceToObj;
double FdistanceToObj;
float pingDistanceCM = 0.0;
double fixedTime = 500;
int state; // (0 = heading hold, 1 = wall following, 2 = drive around box)

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

// servo
Servo steeringServo;
#define servoPin 7 // pin for servo signal
#define SERVOPOT A7
#define SERVOSCALEUS 10
#define SERVOMAXUS 400
double servoAngleDeg;

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

void setup() {
   // current robot state (0 = heading hold, 1 = wall following);
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
}

//////////////////////////////////////////////////////////////////
void loop() {
    double frontDist = getPingDistanceCM(1);
    if (frontDist < 10.0) {
          static double servoCenter_us = 1350.0;  // default value can be changed in SETUP
          servoCenter_us = 800.0 + analogRead(SERVOPOT); 
          wheelTurnBias = constrain(servoCenter_us, servoCenter_us-SERVOMAXUS, servoCenter_us+SERVOMAXUS);
          steeringServo.writeMicroseconds(wheelTurnBias);
          currAngle = 0.0;
          Serial.println(rb);
          Serial.println(wheelTurnBias);
          return;
      }
    wallFollow(1);
    Serial.println(servoAngleDeg);
    setServoAngle(servoAngleDeg);
    digitalWrite(motorFwdPin, HIGH);
    digitalWrite(motorRevPin,LOW);
    analogWrite(motorLPWMPin,motorLPWM);
    analogWrite(motorRPWMPin,motorRPWM);
}

// state = 1 is simpmle wall following, state=2 is drive around box
void wallFollow(int state) {
      
      // get distance to object with left ping sensor
      LdistanceToObj = getPingDistanceCM(0);
      // get distance to object with front ping sensor
      FdistanceToObj = getPingDistanceCM(1);
      // get distance to object with right ping sensor
      RdistanceToObj = getPingDistanceCM(2);
      
      // set speed of motors based on distance to object
      if (LdistanceToObj < 10 || FdistanceToObj < 10 || RdistanceToObj < 10) {
        motorLPWM=0;
        motorRPWM=0;
      } else {
        motorLPWM=100;
        motorRPWM=100;
      }
      dt = micros()-microlast;
      // set servo degree based on distance to object
      static double deltaI = 0.0;
      double error = 30.0 - RdistanceToObj;
      deltaI +=  error * dt / 10000.0;
      servoAngleDeg = constrain((KP*error+KI*deltaI), -30, 30);
      microlast=micros(); 
}

// left ping sensor = 0
// front ping sensor = 1
// right ping sensor = 2
double getPingDistanceCM(int ping) {
  const long timeout_us = 3000;
  if (ping == 0) {
      digitalWrite(LpingTrigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(LpingTrigPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(LpingTrigPin, LOW);
      unsigned long echo_time;
      echo_time = pulseIn(LpingEchoPin, HIGH, timeout_us);
      if (echo_time == 0) {
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
        if (echo_time == 0) {
          echo_time = timeout_us;
        }
        pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);
    } else if (ping == 2) {
        digitalWrite(RpingTrigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(RpingTrigPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(RpingTrigPin, LOW);
        unsigned long echo_time;
        echo_time = pulseIn(RpingEchoPin, HIGH, timeout_us);
        if (echo_time == 0) {
          echo_time = timeout_us;
        }
        pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);
    }
  return pingDistanceCM;
}


void setServoAngle(double sDeg) {
    double ServoCenter_us = wheelTurnBias;
    double ServoScale_us = 10.0;    // micro-seconds per degree
    double t_us = constrain(ServoCenter_us + ServoScale_us * sDeg, ServoCenter_us-200, ServoCenter_us+200);
    steeringServo.writeMicroseconds(t_us);
}
