/*readColour.ino

The following program demonstrates some basic EVNColourSensor functionality.
*/

#include <EVN.h>

#define COL_I2C_PORT 1  //set I2C port for colour sensor here

EVNAlpha board;
EVNColourSensor cs(COL_I2C_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    cs.begin();     //sensor initialization comes after
    Serial.begin(9600);
}

void loop()
{
    int c = cs.read(CLEAR);
    int r = cs.read(RED, false);    //blocking is set to false
    int g = cs.read(GREEN, false);  //to ensure that the sensor does not wait for a new reading
    int b = cs.read(BLUE, false);

    Serial.print("CLEAR:");
    Serial.print(c);
    Serial.print(",RED:");
    Serial.print(r);
    Serial.print(",GREEN:");
    Serial.print(g);
    Serial.print(",BLUE:");
    Serial.println(b);
}