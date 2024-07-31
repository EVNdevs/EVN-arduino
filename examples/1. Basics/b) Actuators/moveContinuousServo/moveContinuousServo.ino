/*moveContinuousServo.ino

The following program demonstrates some basic EVNContinuousServo functionality.
*/

#include <EVN.h>

#define SERVO_PORT 1  //set servo port here

EVNAlpha board;

//EVNAlpha board(BUTTON_TOGGLE, true, true);
//by default, any servos will not stop moving until they are instructed to stop using library functions 
//however, you can set link_movement to true to use the button as an enable/disable switch for motors and servos
//to try this out, uncomment line 11 and comment line 10

EVNContinuousServo servo(SERVO_PORT);

void setup1()
{
    //initialize servo
    //this can be run in void setup(), but running on the second core improves performance
    servo.begin();
}

void setup()
{
    board.begin();  //initialize board at start of void setup()
}

void loop()
{
    //each button press toggles the button's output between "true" and "false" (by default, button outputs "false" on startup)
    //if button outputs "true", run the servo at different speeds and directions for 4 seconds
    if (board.buttonRead())
    {
        servo.write(-100);
        delay(1000);
        servo.write(-50);
        delay(1000);
        servo.write(25);
        delay(1000);
        servo.write(100);
        delay(1000);
    }
    else
    {
        servo.write(0);    //otherwise, stop servo
    }
}