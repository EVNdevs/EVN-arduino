/*bluetoothRemote.ino

The following program demonstrates some basic EVNBluetooth functionality in Remote Mode.
*/

#include <EVN.h>

EVNAlpha board;

EVNBluetooth bt(1, 9600, (char*)"EVN Bluetooth", BT_REMOTE);

void setup()
{
    board.begin();
    bt.begin();

    //prints address and firmware version before exiting Program Mode
    //If connecting to another Bluetooth Module in Host Mode, the address is passed to the constructor for that other module
    if (bt.inProgramMode())
    {
        delay(1000);
        bt.printAddress();
        bt.printVersion();
        bt.exitProgramMode();
    }

    Serial1.begin(9600);
    Serial.begin(9600);
}

void loop()
{
    //this piece of code reads any message received by the Bluetooth Module
    //and prints it to Serial Monitor
    while (Serial1.available())
        Serial.write(Serial1.read());

    //it also writes a message to the connected host (e.g. a computer, smartphone, or another Bluetooth Module)
    Serial1.println("Hello World!");
    delay(1000);

    //however, you can pretty much do anything with Serial1 in void loop()
    //think of it as a wireless Serial connection
}