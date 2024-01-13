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

void loop(){
    sensorCalibrate();
    lineFollow();
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

void lineFollow()
{
    if (isEnableLineFollowing)
    {
        int err = sensor.readSensor();

        int leftMotorSpeed = 130;
        int rightMotorSpeed = 130;

        int difference = (err * kP) + ((err - lasrErr) * kD);

        lasrErr = err;
        motorDrivr(leftMotorSpeed + difference, rightMotorSpeed - difference)
    }
}

void motorDrivr(int leftM, int rightM)
{
    // Left Motor Controll
    if (0 < leftM)
    {
        if (maxSpeed < leftM)
        {
            leftM = maxSpeed;
        }

        digitalWrite(PIN_LEFT_MOTOR_ENABLE, leftM);
        digitalWrite(PIN_LEFT_MOTOR_REAR, LOW);
        digitalWrite(PIN_LEFT_MOTOR_FRONT, HIGH);
    }

    else
    {
        if (leftM < minSpeed)
        {
            leftM = minSpeed;
        }

        digitalWrite(PIN_LEFT_MOTOR_ENABLE, leftM * -1);
        digitalWrite(PIN_LEFT_MOTOR_FRONT, LOW);
        digitalWrite(PIN_LEFT_MOTOR_REAR, HIGH);
    }

    // Right Motor Control
    if (0 < rightM)
    {
        if (maxSpeed < rightM)
        {
            rightM = maxSpeed;
        }

        digitalWrite(PIN_RIGHT_MOTOR_ENABLE, rightM);
        digitalWrite(PIN_RIGHT_MOTOR_REAR, LOW);
        digitalWrite(PIN_RIGHT_MOTOR_FRONT, HIGH);
    }

    else
    {
        if (rightM < minSpeed)
        {
            rightM = minSpeed;
        }

        digitalWrite(PIN_RIGHT_MOTOR_ENABLE, rightM * -1);
        digitalWrite(PIN_RIGHT_MOTOR_FRONT, LOW);
        digitalWrite(PIN_RIGHT_MOTOR_REAR, HIGH);
    }
}