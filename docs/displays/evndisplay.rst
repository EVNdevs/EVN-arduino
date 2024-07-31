``EVNDisplay``
==============

This class provides the following features and functionalities for our OLED Display Standard Peripheral (SSD1315/SSD1306 IC):

    * 8 Rows of Text Output, with 16 Characters per Row
    * Built-in I2C Port Selection and De-selection

This is built upon the u8x8 class from `u8g2`_, a versatile graphics library with many fonts and supported displays.
This library is focused on displaying text, so if you'd like to tinker with animations, consider using this library instead! However, keep in mind that writing to displays over I2C can be slow.

.. _u8g2: https://github.com/olikraus/u8g2

.. note:: This class does I2C port selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

Wiring (I2C)
------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
GND   GND         Ground (0V)
3V3   VCC         3.3V Power
SCL   SCL         I2C Serial Clock
SDA   SDA         I2C Serial Data
====  ==========  ===========

Constructor
-----------
.. class:: EVNDisplay(uint8_t port, bool flip_180deg = false)

    :param port: I2C port the sensor is connected to (1-16)

    :param flip_180deg: Set to ``true`` to rotate display by 180deg. Defaults to ``false``

Functions
---------

.. function:: bool begin()

    Initializes display. Call this function before using the other functions.

    :returns: Boolean indicating whether the display was successfully initialized.

.. function:: void splashEVN()

    Display EVN logo animation for 2 seconds (branding!). Call ``clear()`` afterwards to erase logo.

.. function:: void rotate()

    Rotate display by 180 degrees.

Printing Data to a Row
""""""""""""""""""""""

.. function::   void writeLabel(uint8_t row, data)
                void print(uint8_t row, data)
                

    Prints data at the start of given row.

    :param row: Row to print to (0-7)

    :param data: Data to be printed. Accepts any data type


.. function:: void writeData(uint8_t row, T data)
   
    Prints data to the given row, after any text printed using ``writeLabel()``

    This means that fixed text labels (e.g. "LEFTCOLOUR: ") can be printed once using ``writeLabel()`` in ``void setup()``,
    while the constantly changing data (e.g. colour sensor reading) can be printed using ``writeData()`` in ``void loop()``. 
    This reduces latency compared to rewriting the entire row.

    :param row: Row to print to (0-7)

    :param data: Data to be printed. Accepts any data type

Clearing Rows
"""""""""""""
.. function:: void clear()
    
    Clears all rows on the display

.. function:: void clearLine(uint8_t row)
   
    Clears given row on the display

    :param row: Row to print to (0-7)



