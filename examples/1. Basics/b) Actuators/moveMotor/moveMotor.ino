/*moveMotor.ino

The following program demonstrates some basic EVNMotor functionality.
*/

#include <EVN.h>

#define MOTOR_PORT 1  //set motor port here

EVNAlpha board;

//EVNAlpha board(BUTTON_TOGGLE, true, true);

//by default, any servos will not stop moving until they are instructed to stop using library functions 
//however, you can set link_movement to true to use the button as an enable/disable switch for motors and servos
//to try this out, uncomment line 12 and comment line 10

EVNMotor motor(MOTOR_PORT, EV3_MED);  //motor type can be NXT_LARGE, EV3_LARGE, EV3_MED or CUSTOM_MOTOR

void setup1()
{
	motor.begin(); //initialize motor on 2nd core
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
		//for quick reference, 1 RPM (revolution per minute) = 6 DPS (degrees per second)

		motor.runTime(300, 2000); 	//run motor at 300DPS for 2s
		motor.runAngle(400, -360);	//run motor at -400DPS for -360deg
		motor.runSpeed(-600);    	//run motor at -600DPS until new command is given
		delay(6000);           	//continue running for 6 seconds
	}
	else
	{
		motor.stop();	//otherwise, stop motor
	}
}