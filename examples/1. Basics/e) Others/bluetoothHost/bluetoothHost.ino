/*bluetoothHost.ino

The following program demonstrates some basic EVNBluetooth functionality in Host Mode.
*/

#include <EVN.h>

EVNAlpha board;

//Bluetooth Module is set to use Serial port 2 and run in Host Mode
//Address is obtained from another Bluetooth Module in Remote Mode (see bluetoothRemote example)
EVNBluetooth host(2, 9600, (char*)"EVN Bluetooth Host", BT_HOST, (char*)"19:8:34FE");

void setup()
{
    //initialize board at beginning of void setup()
    board.begin();

    //initialize host
    host.begin();
    Serial2.begin(9600);

    Serial.begin(9600);
}

void loop()
{
    //this piece of code reads any message received by the Bluetooth Host
    //and prints it to Serial Monitor
    while (Serial2.available())
        Serial.write(Serial2.read());

    //however, you can pretty much do anything with Serial1 in void loop()
    //think of it as a wireless Serial connection
}