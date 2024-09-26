#include "EVNCoreSync.h"

//EVNCoreSync0 is used by EVNMotor/EVNDrivebase
//EVNCoreSync1 is used by EVNServo/EVNContinuousServo

EVNCoreSync EVNCoreSync0(CORE1_ISR_WAIT_TIME_US);
EVNCoreSync EVNCoreSync1(CORE1_ISR_WAIT_TIME_US);