#include <BeeLineSensorPro.h>

// PINS FOR MOTORS
#define PIN_LEFT_MOTOR_ENABLE 6
#define PIN_RIGHT_MOTOR_ENABLE 3

#define PIN_LEFT_MOTOR_FRONT 5
#define PIN_LEFT_MOTOR_REAR 7

#define PIN_RIGHT_MOTOR_FRONT 2
#define PIN_RIGHT_MOTOR_REAR 4

// Set Pins of IR_Sensors

BeeLineSensorPro sensor = BeeLineSensorPro((unsigned char[]){
                                               13, A0, A2, A3, A4, A5, 9, 8},
                                           LINE_WHIE);

int maxSpeed = 255;
int minSpeed = -255;

int isEnableLineFollowing = 1;
float kP = 0.05;

float kD = 0.1;
int lasrErr;

void setup()
{
    Serial.begin(9600);

    pinMode(PIN_LEFT_MOTOR_ENABLE, OUTPUT);
    pinMode(PIN_RIGHT_MOTOR_ENABLE, OUTPUT);
    pinMode(PIN_LEFT_MOTOR_FRONT, OUTPUT);
    pinMode(PIN_LEFT_MOTOR_REAR, OUTPUT);
    pinMode(PIN_RIGHT_MOTOR_FRONT, OUTPUT);
    pinMode(PIN_RIGHT_MOTOR_REAR, OUTPUT);
}

void sensorCalibrate()
{
    for (int i = 0; i < 100; i++)
    {
        sensor.calibrate();
        motorDrivr(-130, 130);
    }

    motorDrivr(0, 0);
    delay(150);

    for (int i = 0; i < 200; i++)
    {
        sensor.calibrate();
        motorDrivr(130, -130);
    }

    motorDrivr(0, 0);
    delay(500);
}