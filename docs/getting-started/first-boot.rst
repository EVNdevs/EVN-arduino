First Boot
===============

What you Need
-------------

Here's the list of items you need to get EVN Alpha up and running:

* 1 fully soldered EVN Alpha board
* 2 18650 Cells
* 1 USB 2.0-capable Type C cable for program upload

* 1 5V 3A USB Type-C Power Source for charging

    * Most modern laptops are capable of this, but any charging brick works too

If you have purchased a DIY Kit, refer to our `DIY Guide`_ for soldering instructions and assembly.

.. _DIY Guide: ../guides/diy-v1_4.html

18650 Battery Selection
-----------------------

In EVN Alpha, 2 18650 cells are connected in series for a nominal voltage of 7.4V.

Current Rating
^^^^^^^^^^^^^^
To avoid over-discharge, the 18650 cells must be capable of supplying enough current for all electrical loads on the board.

This includes the 3.3V and 5V regulators which draw max. 5A from the battery, as well as the 4 motor drivers, which are each rated for 3A continuous current with 4A max.

Therefore, each cell would need to be rated for **21A** of maximum discharge current to push the hardware to its limits.


But thanks to the work by `PhiloHome`_, we know the stall current of the NXT and EV3 Motors currently supported:

.. _PhiloHome: https://www.philohome.com/motors/motorcomp.htm

* NXT Large Servo:  2A
* EV3 Large Servo:  1.8A
* EV3 Medium Servo: 0.8A

So if you only intend to use LEGO motors, a current rating of 13A is sufficient (enough for 4 simultaneously stalled NXT motors).

Can I use Button-Top and/or Protected Cells?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**No.** EVN Alpha only supports unprotected flat-top cells, because the cell holders are made for 18650 cells which are 65mm in length, 
so they cannot accomodate button-top or protected cells which are longer than 18650 cells.

You may be able to squeeze in an unprotected button cell or protected flat top cell, but this is **not recommended**, due to the unknown effects
of placing that much force on the cell and holder.

Are the cells protected onboard?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Yes.** EVN Alpha contains the following protection circuits for the cells:

* **Reverse Voltage Protection**: the board and cells are protected if either/both cells are accidentally connected in reverse
* **Undervoltage Lockout (Over-Discharge Protection)**: if the combined voltage dips below 6.3V, the board is locked into charging mode only to prevent excessive over-discharge
* **Overvoltage (Over-Charge) Protection**: charging is suspended when either cell is over-voltage
* **Over-Current & Short-Circuit Protection on All Loads**: Motor Drivers & 3.3+5V Regulators will disable when they are in an over-current or short circuit condition
* **160mA Passive Cell Balancing in Charging Mode**: Prevents cell imbalance from degrading the cells

Other Details
^^^^^^^^^^^^^
Both batteries should have similar capacity, discharge characteristics, state of charge and level of wear (2 new fully-charged batteries of the same model is best).

.. warning::
    
    Remember that proper handling and usage are crucial for safety when dealing with lithium-ion batteries. Always follow manufacturer guidelines and avoid using knock-off or low-quality brands. 

Battery Installation
--------------------
1. Remove the battery cover from the brick. Ensure that the USB-C port is **not** plugged in.

2. There are **2** battery slots in the holder. Inspect the battery polarity markings for each slot on the battery holder and battery cover (which should match each other). Note that the negative terminal of one cell should be near the postive terminal of the other cell, and vice versa.

3. Install the inner cell into place (the battery slot further from the edge of the circuit board). It is recommended to contact the cell's negative terminal with the - contact on the holder first, before pressing the positive end into place.

4. Install the outer cell into place using the same instructions, making sure to orient the cell properly.

5. Firmly press down on both cells to ensure they are well seated in the holder.

Power-On Test
-------------

Once the batteries are inserted, pressing the on/off button (rightmost on the board) should toggle the red Power LED on and off.

If the red Power LED does not turn on, there are 2 possibilities:

* The cells may be inserted in the wrong orientation. Check the polarities by referring to the battery cover, and correct if necessary. Using a screwdriver/stick may be helpful in removing the inner cell.

* The battery voltages are too low. Use a multimeter or battery checker to check if the cells are sufficiently charged. If the combined cell voltage is below 6.3V, EVN Alpha will not power on.

If the board still fails to power on, feel free to reach out on Discord/by email and we'll assist you.


Charging Test
-------------
1. Press the On/Off button to set the board into Off mode. In Off Mode, all LEDs should be unlit.

2. Connect EVN Alpha to your charger using your USB cable. Once plugged in, the green Cable LED should light up.

3. The state of the yellow Charging LED depends on whether the cells are fully charged. If they are, the yellow Charging LED will remain unlit. If the cells are not fully charged, the yellow Charging LED will light up, indicating that everything is working! Feel free to carry on with progr, but remember to check that the board can complete and

The yellow LED blinking indicates a charging error. This could occur for many reasons, including but not limited to:

* USB Input Over-voltage & Under-Voltage
* Battery Not Connected
* Charging IC Temperature Exceeds Safe Limits
* Charging did not Complete after 12 Hours

Feel free to proceed with the following sections first to drain some charge from the cells, but remember to check that the board can successfully complete a charge afterwards.

Board Detection Test
--------------------
1. Press the On/Off button to set the board into On mode. In On Mode, the red Power LED should be lit.

2. Connect EVN Alpha to your computer using your USB cable. Once plugged in, the green Cable LED should light up.

3. Once connected, EVN Alpha will appear as an USB storage device or USB Serial device. A USB Serial device can be hard to detect, so follow these steps to set the board into USB Drive (BOOTSEL) mode:

    * Press and hold Reset Button (next of USB port)
    * Press and hold BOOTSEL Button (next to Reset button)
    * Release Reset Button
    * Release BOOTSEL Button

Following this, EVN Alpha should appear as a USB storage device ready for programming! If it does not, check that your USB cable can transmit data.