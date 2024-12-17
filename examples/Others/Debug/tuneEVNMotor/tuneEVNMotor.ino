#include <EVN.h>

#define MOTOR_PORT 1  //set motor port here

EVNAlpha board;

EVNMotor motor(MOTOR_PORT, EV3_MED, DIRECT);  //motor type can be NXT_LARGE, EV3_LARGE, EV3_MED or CUSTOM_MOTOR

void setup1()
{
    motor.begin();
    motor.setDebug(true);
}

void setup()
{
    board.begin();
}

void loop()
{
    if (board.buttonRead())
    {
        //Serial input format: P 4 digits 2dp + D 4 digits 5dp + Speed(%) 3 digit 0-100
        if (Serial.available() >= 11)
        {
            float kp = 1 * ((float)Serial.read() - 48) + 0.1 * ((float)Serial.read() - 48) + 0.01 * ((float)Serial.read() - 48) + 0.001 * ((float)Serial.read() - 48);
            float kd = 0.01 * ((float)Serial.read() - 48) + 0.001 * ((float)Serial.read() - 48) + 0.0001 * ((float)Serial.read() - 48) + 0.00001 * ((float)Serial.read() - 48);
            float speed = motor.getMaxRPM() * 6 * (100 * ((float)Serial.read() - 48) + 10 * ((float)Serial.read() - 48) + 1 * ((float)Serial.read() - 48)) / 100;
            delay(750);
            motor.setKp(kp);
            motor.setKd(kd);
            motor.runTime(speed, 1500);
            delay(750);
        }
    }
    else
        motor.stop();
}