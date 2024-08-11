/*moveFixedServo.ino

The following program demonstrates some basic EVNServo functionality.
*/

#include <EVN.h>

#define SERVO_PORT 1  //set servo port here

EVNAlpha board;
EVNServo servo(SERVO_PORT);

//EVNAlpha board(BUTTON_TOGGLE, true, true);
//by default, any servos will not stop moving until they are instructed to stop using library functions 
//however, you can set link_movement to true to use the button as an enable/disable switch for motors and servos
//to try this out, uncomment line 11 and comment line 10

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
    //if button outputs "true", run the servo to different positions and speeds for 10 seconds
    if (board.buttonRead())
    {
        servo.write(0, 1000);       //set servo to move to position of 0deg, and wait for 1 second
        servo.write(270, 9500, 20); //set servo to move to position of 270deg at a speed of 20deg per second, and wait for 9.5 seconds
    }
}