#include <Servo.h>

#define PIN_ULTRASONIC_TRIG A8
#define PIN_ULTRASONIC_ECHO A9

#define PIN_SERVO 10

int dangerLowerVertivalLevel = 80;
int properVertivalLevel = 75;
int verticalLevelErrorGap = 10;

float servoToMMRatio = 0.233;
int servoAngle = 0;
Servo servo;

void setup()
{
    Serial.begin(9600);

    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);

    servo.attach(PIN_SERVO);
    servo.write(0);
}