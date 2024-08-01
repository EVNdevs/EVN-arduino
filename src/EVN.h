#ifndef EVN_h
#define EVN_h

#if ( defined(ARDUINO_EVN_ALPHA) )
#else
#error This library is for EVN Alpha using the Arduino-Pico core only! Please check your Tools->Board settings.
#endif

#if (ARDUINO_PICO_MAJOR > 3)
#elif (ARDUINO_PICO_MAJOR == 3 && ARDUINO_PICO_MINOR > 9)
#elif (ARDUINO_PICO_MAJOR == 3 && ARDUINO_PICO_MINOR == 9 && ARDUINO_PICO_REVISION >= 4)
#else
#error Please update Arduino Pico to version >= 3.9.4.
#endif

#include "EVNAlpha.h"

#include "actuators/EVNMotor.h"
#include "actuators/EVNServo.h"             //Geekservo (SERVO)

#include "sensors/EVNColourSensor.h"        //TCS34725
#include "sensors/EVNDistanceSensor.h"      //VL53L0X
#include "sensors/EVNCompassSensor.h"       //HMC5883L
#include "sensors/EVNIMUSensor.h"           //MPU-6500
#include "sensors/EVNEnvSensor.h"           //BME-280
#include "sensors/EVNGestureSensor.h"       //APDS-9960
#include "sensors/EVNTouchArray.h"          //MPR121

#include "displays/EVNRGBLED.h"             //WS2812B (SERVO)
#include "displays/EVNMatrixLED.h"          //HT16K33
#include "displays/EVNSevenSegmentLED.h"    //HT16K33
#include "displays/EVNDisplay.h"            //SSD1306

#include "others/EVNBluetooth.h"            //HC-05 (UART)
#include "others/EVNADC.h"                  //ADS1115

#include "helper/PIDController.h"


#endif