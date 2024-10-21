``EVNRGBLED``
=============

These peripherals use WS2812B RGB LEDs, which feature 3 brightness-controlled channels for Red, Green and Blue.

Normally, a microcontroller would need to transmit a signal of carefully timed HIGHs and LOWs to control the LEDs, which would block the rest of the program due to the use of ``delay()``. 
However, this library uses Programmable IO (PIO) to handle the transmissions, so the main program loop is not blocked when transmitting to the LEDs.

Another thing to note is that these can be daisy-chained, so you can control up to 48 LEDs (6 Peripherals) with one Servo port!

.. note:: Each instance of EVNRGBLED (4 max) consumes 1 of the RP2040's 8 PIO state machines, so keep this in mind if you are using PIO for your own purposes.

.. note:: EVNRGBLED consumes some of the RP2040's DMA peripherals. See this `page`_ for more info.

.. _page: ../getting-started/hardware-overview.html

Wiring (Servo)
--------------

====  ==========   ===========
Host  Peripheral   Description
====  ==========   ===========
SIG   IN           Signal
5V    VCC          5V Power
GND   GND          Ground (0V)
====  ==========   ===========

Chaining Multiple RGB LEDs
--------------------------

It's possible to chain up to 6 RGB LED Peripherals in a row without needing to use more Servo ports.

Simply connect the OUT of one Peripheral to the IN pin of the next one in the chain.

==========  ===============   ===========
Peripheral  Next Peripheral   Description
==========  ===============   ===========
OUT         IN                Signal
VCC         VCC               5V Power
GND         GND               Ground (0V)
==========  ===============   ===========

.. warning:: Each RGB LED consumes 60mA of current at max brightness. Using 6 sticks (48 RGB LEDs) consumes 2.88A maximum, which is close to the 3A limit of the Servo ports. Keep this in mind if you have other peripherals powered by Servo ports!

Constructor
-----------

.. class:: EVNRGBLED(uint8_t port, uint8_t led_count = 8, bool invert = false)

    :param port: Servo port (1-4)
    :param led_count: Number of LEDs connected to the servo port. Defaults to 8
    :param invert: Whether order of LED indexes should be inverted

Functions
---------

.. function:: bool begin()

    Initializes RGB LED Array. Call this function before using the other functions.

    :returns: Boolean indicating whether the RGB LED array was successfully initialized.

.. function:: void end()

    Deinitializes RGB LED Array and releases all PIO state machines and DMA peripherals consumed by it. 
    Does not clear the data currently written to the RGB LED Array.

Set Functions
""""""""""""""
.. function:: void setLEDCount(uint8_t led_count)

    :param led_count: Number of LEDs in array

.. function:: void setInvert(bool enable)

    By default, the LED with index 0 is the one closest to the IN wire. 
    When enabled (set to ``true``), the index order is inverted, so that index 0 is the LED furthest from the IN wire.

    :dir: Whether LED index is inverted

Get Functions
""""""""""""""
.. function:: uint8_t getLEDCount()

    :returns: number of LEDs in the array

.. function:: bool getInvert()

    :returns: Whether LED index is inverted

Display Functions
-----------------

.. function:: void writeOne(uint8_t led, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, bool show = true)

    Updates given LED's RGB values in buffer and if ``show`` is ``true``, writes buffer to peripheral.

    :param led: Index of LED to update (0 to (led_count-1)). LED 0 is the LED closest to the signal & power pins
    :param r: Red channel intensity (0-255). Defaults to 0
    :param g: Green channel intensity (0-255). Defaults to 0
    :param b: Blue channel intensity (0-255). Defaults to 0
    :param show: Whether to write buffer to LEDs. Defaults to ``true``

.. function:: void clearOne(uint8_t led, bool show = true)

    Set given LED to turn off in buffer and if ``show`` is ``true``, writes buffer to peripheral.

    :param led: Index of LED to update (0 to (led_count-1)). LED 0 is the LED closest to the signal & power pins
    :param show: Whether to write buffer to LEDs. Defaults to ``true``

.. function:: void writeLine(uint8_t start_led, uint8_t end_led, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, bool show = true)

    Update given range of LEDs' RGB values in buffer and if ``show`` is ``true``, writes buffer to peripheral.

    :param start_led: Starting index of LED to update (0 to (led_count-1)). LED 0 is the LED closest to the signal & power pins
    :param end_led: Ending index of LED to update (0 to (led_count-1)). This LED will be updated as well.
    :param r: Red channel intensity (0-255). Defaults to 0
    :param g: Green channel intensity (0-255). Defaults to 0
    :param b: Blue channel intensity (0-255). Defaults to 0
    :param show: Whether to write buffer to LEDs. Defaults to ``true``

.. function:: void writeLine(uint8_t start_led, uint8_t end_led, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, bool show = true)

    Update given range of LEDs to turn off in buffer and if ``show`` is ``true``, writes buffer to peripheral.

    :param start_led: Starting index of LED to update (0 to (led_count-1)). LED 0 is the LED closest to the signal & power pins
    :param end_led: Ending index of LED to update (0 to (led_count-1)). This LED will be updated as well.
    :param show: Whether to write buffer to LEDs. Defaults to ``true``

.. function:: void writeAll(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, bool show = true)

    Updates all LEDs' RGB values in buffer and if ``show`` is ``true``, writes buffer to peripheral.

    :param r: Red channel intensity (0-255). Defaults to 0
    :param g: Green channel intensity (0-255). Defaults to 0
    :param b: Blue channel intensity (0-255). Defaults to 0
    :param show: Whether to write buffer to LEDs. Defaults to ``true``

.. function:: void clearAll(bool show = true)

    Set all LEDs to turn off in buffer and if ``show`` is ``true``, writes buffer to peripheral, essentially turning all LEDs off.

    :param show: Whether to write buffer to LEDs. Defaults to ``true``

.. function:: void update()

    Writes buffer to LEDs. 

    By default, this function is internally called at the end of all the ``write###()`` / ``clear###()`` functions.

    However, it may be more useful for you to make multiple changes to the buffer using the above functions 
    with ``show`` set to ``false``, before calling ``show()`` at the end to update the LEDs. 

    This approach can achieve nicer visual output (all pixels change simultaneously) or faster updates (as ``update()`` is only called once).