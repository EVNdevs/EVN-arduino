//TUNING FOR CUSTOM MOTOR
//edit this to change CUSTOM_MOTOR defaults
#define CUSTOM_KP                       0.3     //position PID gains (PD controller)
#define CUSTOM_KD		                0.0
#define CUSTOM_PWM_MAG                  -1
#define CUSTOM_PWM_EXP                  -1
#define CUSTOM_UNLOADED_MAX_RPM	        100     //max RPM of the motor shaft
#define CUSTOM_LOADED_MAX_RPM   		100
#define CUSTOM_ACCEL                    10000   //acceleration in rpm/s (60 means it takes 1 second to accelerate from 0RPM to 60)
#define CUSTOM_DECEL                    10000   //deceleration in rpm/s (60 means it takes 1 second to decelerate from 60RPM to 0)
#define CUSTOM_PPR				        500     //pulses on 1 encoder wheel for 1 revolution of the motor shaft

//LEGO MOTORS
#define LEGO_PPR				        180

//TUNING FOR LEGO EV3 LARGE SERVO MOTOR
#define EV3_LARGE_KP                    0.212
#define EV3_LARGE_KD	                0.00403
#define EV3_LARGE_PWM_MAG               0.214
#define EV3_LARGE_PWM_EXP               1.5304
#define EV3_LARGE_UNLOADED_MAX_RPM		155
#define EV3_LARGE_LOADED_MAX_RPM		155
#define EV3_LARGE_ACCEL                 155 * 600
#define EV3_LARGE_DECEL                 155 * 600

//TUNING FOR LEGO NXT LARGE SERVO MOTOR
#define NXT_LARGE_KP	                0.188
#define NXT_LARGE_KD	                0.00250
#define NXT_LARGE_PWM_MAG               0.5544
#define NXT_LARGE_PWM_EXP               0.612
#define NXT_LARGE_UNLOADED_MAX_RPM		155
#define NXT_LARGE_LOADED_MAX_RPM		155
#define NXT_LARGE_ACCEL                 155 * 600
#define NXT_LARGE_DECEL                 155 * 600

//TUNING FOR LEGO EV3 MEDIUM SERVO MOTOR
#define EV3_MED_KP	                    0.164
#define EV3_MED_KD	                    0.00138
#define EV3_MED_PWM_MAG                 0.3962
#define EV3_MED_PWM_EXP                 0.8718
#define EV3_MED_UNLOADED_MAX_RPM		245
#define EV3_MED_LOADED_MAX_RPM			245
#define EV3_MED_ACCEL                   245 * 600
#define EV3_MED_DECEL                   245 * 600

//TUNING FOR EVNMOTOR CLASS
#define MOTOR_POS_MIN_ERROR_MOTOR_DEG   0.5
#define MOTOR_POS_TIMEOUT_US            250000
#define MOTOR_HOLD_MIN_ERROR_MOTOR_DEG  0.5
#define MOTOR_HOLD_THRESHOLD_DPS        10
#define MOTOR_HOLD_MIN_TIME_US          50000
#define MOTOR_HOLD_TIMEOUT_US           250000

//TUNING FOR EVNDRIVEBASE CLASS
#define DRIVEBASE_POS_MIN_ERROR_MOTOR_DEG   5
#define DRIVEBASE_POS_TIMEOUT_US            250000
#define DRIVEBASE_HOLD_MIN_ERROR_MOTOR_DEG  5
#define DRIVEBASE_HOLD_THRESHOLD_DPS        10
#define DRIVEBASE_HOLD_MIN_TIME_US          50000
#define DRIVEBASE_HOLD_TIMEOUT_US           250000

#define DRIVEBASE_SPEED_ACCEL               1000
#define DRIVEBASE_SPEED_DECEL               1000
#define DRIVEBASE_TURN_RATE_ACCEL           1000
#define DRIVEBASE_TURN_RATE_DECEL           1000