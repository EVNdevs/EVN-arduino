#include "EVNCoreSync.h"

//EVNCoreSync0 is used by EVNMotor/EVNDrivebase
//EVNCoreSync1 is used by EVNServo/EVNContinuousServo

EVNCoreSync EVNCoreSync0(CORE1_ISR_SPIN_TIMEOUT_US, CORE1_ISR_STALL_TIME_US);
EVNCoreSync EVNCoreSync1(CORE1_ISR_SPIN_TIMEOUT_US, CORE1_ISR_STALL_TIME_US);