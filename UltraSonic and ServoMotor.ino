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

void loop()
{
    long verticalLevel = getUltrasonicInMM();

    if (verticalLevel < dangerLowerVertivalLevel)
    {
        if ((verticalLevel < properVertivalLevel + verticalLevelErrorGap) || (verticalLevel > properVertivalLevel - verticalLevelErrorGap))
        {
            servoAngle += (properVertivalLevel - verticalLevel) * servoToMMRatio;
            ServoRotate(servoAngle); // change verticalLevel of blade
            delay(400);
        }
    }
    else
    {
        Serial.write("near to a slant");
    }
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