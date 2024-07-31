#include <EVN.h>

EVNAlpha board;
EVNMotor left(2, EV3_MED, REVERSE), right(3, EV3_MED);

void setup()
{
    board.begin();
    left.begin();
    right.begin();

    Serial.begin(9600);
    Serial1.begin(9600);
}

void loop()
{
    if (Serial1.available())
    {
        int data = Serial1.read();

        Serial.println(data);

        switch (data) {
        case 0: //forward
            left.runSpeed(100);
            right.runSpeed(100);
            break;

        case 1: //left
            left.runSpeed(-60);
            right.runSpeed(60);
            break;

        case 2: //right
            left.runSpeed(60);
            right.runSpeed(-60);
            break;

        case 3: //backward
            left.runSpeed(-100);
            right.runSpeed(-100);
            break;

        case 4: //stop
            left.stop();
            right.stop();
            break;
        }
    }
}