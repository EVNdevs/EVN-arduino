Programming
============

Start by opening EVN's ButtonLED example sketch in Arduino (File > Examples > EVN > 01.Basics > buttonLED).

Board Selection
"""""""""""""""

To upload to EVN Alpha, select EVN Alpha as your board (Tools > Board > Raspberry Pi RP2040 Boards > EVNAlpha). Setting the right board ensures that the USB Serial connection and upload process are stable.

USB Drive (BOOTSEL) Mode
"""""""""""""""""""""""""

Next, we will set the board to appear as a USB Mass Storage Drive to the computer, ready for upload. RP2040 and other Pico-related docs call this BOOTSEL mode, but we prefer to call it USB Drive mode.

In this mode, any previously uploaded programs are also stopped from running.

There are 2 ways to set the board into USB Drive mode:

1. Reset and BOOTSEL (Faster)

    * Press and Hold the Reset Button
    * Press and Hold the BOOTSEL Button
    * Release the Reset Button
    * Release the BOOTSEL Button

2. Power and BOOTSEL (Easier)

    * Set the board to Off mode using On/Off Button
    * Press and Hold the BOOTSEL Button
    * Set the board to On mode using On/Off Button
    * Release the BOOTSEL Button

Now, the board should appear as a USB Drive when connected to your computer, and it will appear as a UF2 Board option in Tools > Port. Select it, and upload your code.

After your new code has been uploaded, look through the available ports in Tools > Port again. If the code runs properly, a COM Port with a label (e.g. COM7 (Raspberry Pi Pico W)) should be available for you to upload with and access on Serial Monitor.

Uploading
---------

After selecting the correct board, press the Upload button (right arrow) in the top left corner. For fresh installs of the Arduino-Pico core and EVN library, the first compile may take a while, but subsequent uploads should take less time.

When you press the User Button (leftmost one), the blue User LED should toggle on and off!

COM Port Selection
------------------

Once the code has been uploaded, the board will cease to appear as a USB Drive. Instead, under Tools > Port, 
you should be able to see a COM Port available with a label (e.g. COM3 (Raspberry Pi Pico W)). Select it, and you will be able to communicate with the program using the Serial Monitor.

.. note:: If a COM Port is not available, it may be due to incorrect Board Selection settings, or a crashed program. Set the board to USB Drive mode again, and try to upload a different program.

Writing Your First Sketch
-------------------------
To start, include the EVN library using #include <EVN.h> in your program.

All EVN sketches will use the board class EVNAlpha for built-in functions. Aside from this class, there are classes for each of our Standard Peirpherals.

This is what an example sketch with 2 motors and 2 colour sensors would look like:

.. code-block:: cpp

    #include <EVN.h>

    EVNAlpha board;
    EVNMotor motor_left(3, EV3_LARGE),
             motor_right(2, EV3_LARGE);

    EVNColourSensor cs1(1), cs2(2);

    void setup()
    {
        //board.begin() is always at the start of void setup()
        board.begin();

        cs1.begin();
        cs2.begin();
    }

    void setup1()
    {
        //motor begin can be called on 1st core, but calling on 2nd core improves performance
        motor_left.begin();
        motor_right.begin();
    }

    void loop()
    {
        int cs1_reading = cs1.readClear();
        int cs2_reading = cs2.readClear();

        //do something with colour sensor readings here

        motor_left.runSpeed(600);
        motor_right.runSpeed(600);
    }


More examples can be found in the EVN library examples (File > Examples > EVN...).

.. note:: Example sketches are a little sparse right now, we hope to add more as soon as we can

Uploading Shortcuts
-------------------

The basic process of uploading is listed as above:

1. Set board settings (if not already set)
2. Set board to USB Drive mode
3. Select UF2 Board in Tools > Port
4. Upload Code
5. Select COM Port to use Serial Monitor

The Arduino-Pico core does introduce some ways to make this process faster:

* After the upload button is pressed, the IDE should automatically detect any board in USB Drive mode and upload to it, making step 3 unnecessary

* If the board is not in USB Drive mode, but its program has not crashed and the board is connected with its COM Port correctly set **before** the upload button is pressed, 
    the IDE should automatically detect the board and upload to it, making steps 2 and 3 unnecessary

* If you add rp2040.enableDoubleResetBootloader() inside void setup()/ void setup1() or void loop()/ void loop1(), pressing the Reset button twice will set the board to USB Drive mode, making step 2 much easier.
    Getting the correct timing for this may take a few tries; it will not enter USB Drive mode if double-tapped too quickly or too slowly.

Since these "shortcuts" can fail for a variety of reasons (crashed code, USB inconsistencies from computer to computer or different operating systems),
we cannot endorse them as foolproof upload methods that work 100% of the time. So if they do fail, follow the basic uploading process and everything should work.