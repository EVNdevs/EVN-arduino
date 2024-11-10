//TUNING FOR CUSTOM MOTOR
//edit this to change CUSTOM_MOTOR defaults
#define KP_CUSTOM		        0.25       //position PID gains (PD controller)
#define KI_CUSTOM	        	0.000125
#define KD_MAX_CUSTOM		    5
#define CUSTOM_MOTOR_MAX_RPM	77      //max RPM of the motor shaft
#define CUSTOM_MOTOR_ACCEL      7700    //deceleration in rpm/s (60 means it takes 1 second to accelerate from 0RPM to 60)
#define CUSTOM_MOTOR_DECEL      7700    //deceleration in rpm/s (60 means it takes 1 second to decelerate from 60RPM to 0)
#define CUSTOM_PPR				1000    //pulses on 1 encoder wheel for 1 revolution of the motor shaft

//LEGO MOTORS
#define LEGO_PPR				180

//TUNING FOR LEGO EV3 LARGE SERVO MOTOR
#define KP_EV3_LARGE            0.35
#define KI_EV3_LARGE	        0.00035
#define KD_MAX_EV3_LARGE	    2.45
#define EV3_LARGE_MAX_RPM		155
#define EV3_LARGE_ACCEL         155 * 600
#define EV3_LARGE_DECEL         155 * 600

//TUNING FOR LEGO EV3 MEDIUM SERVO MOTOR
#define KP_EV3_MED	            0.16
#define KI_EV3_MED	            0.00016
#define KD_MAX_EV3_MED	        1.3
#define EV3_MED_MAX_RPM			245
#define EV3_MED_ACCEL           245 * 600
#define EV3_MED_DECEL           245 * 600

//TUNING FOR LEGO NXT LARGE SERVO MOTOR
#define KP_NXT_LARGE	        0.35
#define KI_NXT_LARGE	        0.00035
#define KD_MAX_NXT_LARGE	    2.45
#define NXT_LARGE_MAX_RPM		155
#define NXT_LARGE_ACCEL         155 * 600
#define NXT_LARGE_DECEL         155 * 600

//TUNING FOR EVNMOTOR CLASS
#define USER_RUN_DEGREES_MIN_ERROR_MOTOR_DEG    0.5

//TUNING FOR EVNDRIVEBASE CLASS
#define USER_DRIVE_POS_MIN_ERROR_MOTOR_DEG      2

#define DRIVEBASE_KP_SPEED                  6.25
#define DRIVEBASE_KI_SPEED                  0.0000625
#define DRIVEBASE_KD_SPEED                  3.125

#define DRIVEBASE_KP_TURN_RATE              6.25
#define DRIVEBASE_KI_TURN_RATE              0.00003125
#define DRIVEBASE_KD_TURN_RATE              12.5

#define USER_SPEED_ACCEL                    500
#define USER_SPEED_DECEL                    500
#define USER_TURN_RATE_ACCEL                500
#define USER_TURN_RATE_DECEL                500