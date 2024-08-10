/*customI2CDevice.ino

The following program uses some example code on how to use EVN's multiplexed I2C Ports with non-Standard Peripherals.
As an example, it uses 2 VL53L0X instances and example code from Pololu's VL53L0X library.

However, feel free to use this code to refer to the functions marked EVN ONLY
Adding these functions will allow you to interface with any I2C device
*/

#include <EVN.h>
#include <VL53L0X.h>

#define SENSOR_PORT 1
#define SENSOR2_PORT 9

EVNAlpha board;
VL53L0X sensor, sensor2;

void setup()
{
	//---EVN ONLY---
	board.begin(); 	//initialize board
	//--------------

	Serial.begin(9600);
	Wire.begin();

	//---EVN ONLY---
	//select the port of the sensor to be used, before sending commands
	board.setPort(SENSOR_PORT);
	//--------------

	sensor.setTimeout(500);
	if (!sensor.init())
	{
		Serial.println("Failed to detect and initialize sensor!");
		while (1) {}
	}
	sensor.startContinuous();

	//---EVN ONLY---
	//Ports 9-16 run on the second I2C Bus (Wire1 in software)
	//sensor2 uses Port 9, so it must be set to run on Wire1
	board.setPort(SENSOR2_PORT);
	sensor2.setBus(&Wire1);
	//-------------

	sensor2.setTimeout(500);
	if (!sensor2.init())
	{
		Serial.println("Failed to detect and initialize sensor!");
		while (1) {}
	}
	sensor2.startContinuous();


}

void loop()
{
	//---EVN ONLY---
	board.setPort(SENSOR_PORT);
	//--------------

	Serial.print(sensor.readRangeContinuousMillimeters());
	if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

	//---EVN ONLY---
	board.setPort(SENSOR2_PORT);
	//--------------

	Serial.print(" ");
	Serial.print(sensor2.readRangeContinuousMillimeters());
	if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

	Serial.println();
}