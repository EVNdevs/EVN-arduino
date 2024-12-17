#include <EVN.h>

#define MOTOR_PORT_1 1  //set 1st motor port here
#define MOTOR_PORT_2 4  //set 2nd motor port here

EVNAlpha board;

EVNMotor left(MOTOR_PORT_1, EV3_LARGE, DIRECT);
EVNMotor right(MOTOR_PORT_2, EV3_LARGE, DIRECT);
EVNDrivebase drivebase(31.2, 170, &left, &right);

void setup1()
{
    left.begin();
    right.begin();
    drivebase.begin();
    drivebase.setDebug(DEBUG_SPEED);
}

void setup()
{
    board.begin();  //initialize board at start of void setup()
}

void loop()
{
    if (board.buttonRead())
    {
        //Serial command format: P 4 digits 0dp + D 4 digits 3dp + Speed(%) 3 digits 0-100
        if (Serial.available() >= 11)
        {
            float kp = 1000 * ((float)Serial.read() - 48) + 100 * ((float)Serial.read() - 48) + 10 * ((float)Serial.read() - 48) + 1 * ((float)Serial.read() - 48);
            float kd = 1 * ((float)Serial.read() - 48) + 0.1 * ((float)Serial.read() - 48) + 0.01 * ((float)Serial.read() - 48) + 0.001 * ((float)Serial.read() - 48);
            float speed = drivebase.getMaxSpeed() * (1 * ((float)Serial.read() - 48) + 0.1 * ((float)Serial.read() - 48) + 0.01 * ((float)Serial.read() - 48));
            delay(750);
            drivebase.setSpeedKp(kp);
            drivebase.setSpeedKd(kd);
            drivebase.drive(speed, 0);
            delay(1500);
            drivebase.stop();
            delay(750);
        }
    }
    else
        drivebase.stop();
}