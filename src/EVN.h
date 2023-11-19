#ifndef EVN_h
#define EVN_h

#include "EVNMotor.h"
#include "EVNButton.h"
#include "EVNDisplay.h"
#include "EVNPortSelector.h"
#include "PIDController.h"
#include "RPi_Pico_ISR_Timer.h"

#define calibrate(reading, low, high) ((reading - low) / (high - low))

#endif