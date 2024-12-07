/*moveDrivebase.ino

The following program demonstrates some basic EVNDrivebase functionality.
*/

#include <EVN.h>

#define MOTOR_PORT_LEFT 1  //set left motor port here
#define MOTOR_PORT_RIGHT 4  //set right motor port here

EVNAlpha board;

//EVNAlpha board(BUTTON_TOGGLE, true, true);

//by default, any servos will not stop moving until they are instructed to stop using library functions 
//however, you can set link_movement to true to use the button as an enable/disable switch for motors and servos
//to try this out, uncomment line 12 and comment line 10

EVNMotor left(MOTOR_PORT_LEFT, EV3_LARGE);  //motor type can be NXT_LARGE, EV3_LARGE, EV3_MED or CUSTOM_MOTOR
EVNMotor right(MOTOR_PORT_RIGHT, EV3_LARGE);  //motor type can be NXT_LARGE, EV3_LARGE, EV3_MED or CUSTOM_MOTOR
EVNDrivebase db(62.4, 170, &left, &right);

void setup1()
{
    //initialize motors AND drivebase on 2nd core
    left.begin();
    right.begin();
    db.begin();
}

void setup()
{
    board.begin();  //initialize board at start of void setup()
}

void loop()
{
    //each button press toggles the button's output between "true" and "false" ("false" upon startup)
    //if button outputs "true", run the motor functions
    if (board.buttonRead())
    {
        db.drive(0, 75);
        delay(5000);
        db.straight(50, 200);
        db.curve(50, 100, 90);
    }
    else
    {
        db.stop();	//otherwise, stop drivebase
    }
}