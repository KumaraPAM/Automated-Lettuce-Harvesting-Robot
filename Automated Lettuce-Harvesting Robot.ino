#include <BeeLineSensorPro.h>
#include <Servo.h>

// PINS FOR MOTORS
#define PIN_LEFT_MOTOR_ENABLE 2
#define PIN_LEFT_MOTOR_FRONT 3
#define PIN_LEFT_MOTOR_REAR 4

#define PIN_RIGHT_MOTOR_ENABLE 6
#define PIN_RIGHT_MOTOR_FRONT 7
#define PIN_RIGHT_MOTOR_REAR 8

// PINS FOR MOTORS
#define PIN_ULTRASONIC_TRIG A8
#define PIN_ULTRASONIC_ECHO A9

#define PIN_SERVO 10

// Set Pins of IR_Sensors

BeeLineSensorPro sensor = BeeLineSensorPro((unsigned char[]){
                                               A0, A1, A2, A3, A4, A5, A6, A6, A7},
                                           LINE_WHIE);

int maxSpeed = 255;
int minSpeed = -255;

int isEnableLineFollowing = 1;
float kP = 0.05;

float kD = 0.1;
int lasrErr;

int dangerLowerVertivalLevel = 80;
int properVertivalLevel = 75;
int verticalLevelErrorGap = 10;

float servoToMMRatio = 0.233;
int servoAngle = 0;
Servo servo;

