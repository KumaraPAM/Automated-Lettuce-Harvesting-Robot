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

float servoToMMRatio = 0.2;
int servoAngle = 0;
Servo servo;
void setup()
{
    Serial.begin(9600);

    pinMode(PIN_LEFT_MOTOR_ENABLE, OUTPUT);
    pinMode(PIN_RIGHT_MOTOR_ENABLE, OUTPUT);
    pinMode(PIN_LEFT_MOTOR_FRONT, OUTPUT);
    pinMode(PIN_LEFT_MOTOR_REAR, OUTPUT);
    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);

    servo.attach(PIN_SERVO);
    servo.write(0);

    sensorCalibrate();
}

void loop()
{
    long verticalLevel = getUltrasonicInMM();

    if (verticalLevel < dangerLowerVertivalLevel)
    {
        isEnableLineFollowing = 1;

        if ((verticalLevel < properVertivalLevel + verticalLevelErrorGap) || (verticalLevel > properVertivalLevel - verticalLevelErrorGap))
        {
            servoAngle += (properVertivalLevel - verticalLevel) * servoToMMRatio;
            ServoRotate(servoAngle); // change verticalLevel of blade
            delay(400);
        }
    }
    else
    {
        isEnableLineFollowing = 0;
        Serial.write("near to a fall");
    }

    lineFollow();
}

void ServoRotate(int r)
{
    if (r < 0)
    {
        r = 0;
    }
    if (r > 180)
    {
        r = 180;
    }

    servo.write(r);
}

long getUltrasonicInMM()
{
    long duration;

    delay(5);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    delayMicroseconds(15);
    duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 180000UL);

    return (duration / 2) / 2.91;
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
