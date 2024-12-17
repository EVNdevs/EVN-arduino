/*customMotor.ino

This program shows how to use the EVNMotor class for non-LEGO motors
*/

#include <EVN.h>

#define MOTOR_PORT 1  //set motor port here

EVNAlpha board;

// By default, setting the motor type to CUSTOM_MOTOR
// The motor profile contains the following settings
// * PPR
// * Max RPM
// * Max Acceleration / Deceleration
// * PD Tuning
// * PWM Mapping

EVNMotor motor(MOTOR_PORT, CUSTOM_MOTOR);

// These settings can be edited in the library folder:
// <library folder>/src/evn_motor_defs.h (under TUNING FOR CUSTOM MOTOR)
// However, they can also be changed in code, as we will show below

void setup1()
{
    motor.begin(); //initialize motor on 2nd core

    // These define the motor and encoder characteristics
    // They can typically be found from the motor's product page
    motor.setPPR(1000);
    motor.setMaxRPM(1000);

    // These 2 settings determine how quickly your motor can accelerate/decelerate from rest to maximum speed
    // If you want your motor to accelerate as quickly as possible, you can set them to a very high value
    motor.setDecel(10000);
    motor.setAccel(10000);

    // See https://evn.readthedocs.io/guides/motor for guides on obtaining a PWM mapping / PD tuning for your motor

    // The PWM mapping models the response of the motor when there is no load on its shaft
    // This can be helpful, since PWM is not linear to motor speed (10% PWM duty cycle != 10% no-load speed)
    // However, it's not strictly necessary; set either input to -1 to disable
    motor.setPWMMapping(-1, -1);

    // These settings control the motor's speed/position controller
    motor.setKp(0.3);
    motor.setKd(0.003);
}

void setup()
{
    board.begin(); //initialize board at start of void setup()
}

void loop()
{
    //each button press toggles the button's output between "true" and "false" ("false" upon startup)
    if (board.buttonRead())
    {
        //for quick reference, 1 RPM (revolution per minute) = 6 DPS (degrees per second)

        motor.runTime(300, 3000); 	//run motor at 300DPS for 3s
        motor.runAngle(400, -360);	//run motor at 400DPS for -360deg (runAngle(-100, 360) produces the same effect)
        motor.runSpeed(-600);    	//run motor at -600DPS until new command is given
        delay(10000);           	//continue running for 10 seconds
    }
    else
    {
        motor.stop();	//otherwise, stop motor
    }
}