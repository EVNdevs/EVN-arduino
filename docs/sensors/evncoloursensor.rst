``EVNColourSensor``
===================

This class provides the following features and functionalities for our Colour Sensor Standard Peripheral (TCS34725 IC):

    * Reflected Red, Green, Blue (RGB) & Clear Light Measurements
    * Non-Blocking & Blocking Read Functions
    * Configurable Gain and Integration Times
    * Built-in I2C Port Selection and De-selection

.. note:: This class does I2C port selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

Wiring (I2C)
------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
 --   VIN         Not Soldered
GND   GND         Ground (0V)
3V3   3V3         3.3V Power
SCL   SCL         I2C Serial Clock
SDA   SDA         I2C Serial Data
 --   INT         Not Connected
 --   LED         Not Soldered
====  ==========  ===========

Constructor
-----------

.. class:: EVNColourSensor(uint8_t port, uint8_t integration_cycles = 1, gain gain = gain::X16)

    The listed default settings for the sensor are listed here. Refer to ``setGain()`` & ``setIntegrationTime()`` for more information.

    :param port: I2C port the sensor is connected to (1-16)
    :param integration_cycles: Number of 2.4ms integration cycle for one reading. Defaults to 1
    :param gain: Gain applied to readings. Defaults to ``EVNColourSensor::gain::X16``

Functions
---------

.. function:: bool begin()

    Initializes colour sensor. Call this function before using the other functions.

    :returns: Boolean indicating whether the sensor was successfully initialized. If ``false`` is returned, all other functions will return 0.

.. note::
    If ``begin()`` returns ``false``, try checking your connections and I2C port number!

.. function:: void setGain(gain gain)

    Sets internal sensor gain applied to reading. Increasing gain widens the numerical range of readings at the cost of noise.

    :param gain: Gain applied to readings

        * ``EVNColourSensor::gain::X1`` -- 1x gain
        * ``EVNColourSensor::gain::X4`` -- 4x gain
        * ``EVNColourSensor::gain::X16`` -- 16x gain
        * ``EVNColourSensor::gain::X60`` -- 60x gain

.. function:: void setIntegrationCycles(uint8_t integration_cycles)

    Sets the number of 2.4ms integration cycles used for 1 reading.

    integration time = 2.4ms * integration cycles

    :param integration_cycles: Number of 2.4ms integration cycles for 1 reading (1-255)


.. function:: void writeSettings(uint8_t integration_cycles, gain gain)

    Sets the number of 2.4ms integration cycles used for 1 reading and internal sensor gain.

    :param integration_cycles: Number of 2.4ms integration cycles for 1 reading

    :param gain: Gain applied to readings

        * ``EVNColourSensor::gain::X1`` -- 1x gain
        * ``EVNColourSensor::gain::X4`` -- 4x gain
        * ``EVNColourSensor::gain::X16`` -- 16x gain
        * ``EVNColourSensor::gain::X60`` -- 60x gain

Reading Raw RGBC Values
"""""""""""""""""""""""

.. function::   uint16_t readClear(bool blocking = true)

    Returns raw clear reading from sensor.

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw clear reading

.. function:: uint16_t readRed(bool blocking = true)

    Returns raw red reading from sensor.

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw red reading

.. function:: uint16_t readGreen(bool blocking = true)

    Returns raw green reading from sensor.

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw green reading

.. function:: uint16_t readBlue(bool blocking = true)

    Returns raw blue reading from sensor.

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw blue reading

Reading Calibrated RGBC Values
"""""""""""""""""""""""""""""""
Before reading calibrated values, you need to call the ``setXXrange()`` function for a given colour channel first.

.. function:: void setClearRange(uint16_t low, uint16_t high)
    
    Sets the range of possible clear values used for calibrating raw values.

    Calibrated Clear Reading = (Clear Raw - Clear Low) / (Clear High - Clear Low)

    If this function is not called, ``readClearNorm()`` returns 0;

    :param low: lower bound of readings for Clear channel

    :param high: upper bound of readings for Clear channel

.. function:: void setRedRange(uint16_t low, uint16_t high)
    
    Sets the range of possible red values used for calibrating raw values.

    Calibrated Red Reading = (Red Raw - Red Low) / (Red High - Red Low)

    If this function is not called, ``readRedNorm()`` returns 0;

    :param low: lower bound of readings for Red channel

    :param high: upper bound of readings for Red channel

.. function:: void setGreenRange(uint16_t low, uint16_t high)
    
    Sets the range of possible green values used for calibrating raw values.

    Calibrated Green Reading = (Green Raw - Green Low) / (Green High - Green Low)

    If this function is not called, ``readGreenNorm()`` returns 0;

    :param low: lower bound of readings for Green channel

    :param high: upper bound of readings for Green channel

.. function:: void setBlueRange(uint16_t low, uint16_t high)
    
    Sets the range of possible blue values used for calibrating raw values.

    Calibrated Blue Reading = (Blue Raw - Blue Low) / (Blue High - Blue Low)

    If this function is not called, ``readBlueNorm()`` returns 0;

    :param low: lower bound of readings for Blue channel

    :param high: upper bound of readings for Blue channel

After calling these functions, you can use the ``readXXNorm()`` functions.

.. function:: float readClearCal(bool blocking = true)

    Returns calibrated clear reading from sensor.

    Calibrated Clear Reading = (Clear Raw - Clear Low) / (Clear High - Clear Low)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns:

        * if ``setClearRange()`` has been called, calibrated clear reading from 0 to 1
        * -1 otherwise

.. function:: float readRedCal(bool blocking = true)
    
    Returns calibrated red reading from sensor.

    Calibrated Red Reading = (Red Raw - Red Low) / (Red High - Red Low)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns:

        * if ``setRedRange()`` has been called, calibrated red reading from 0 to 1
        * -1 otherwise

.. function:: float readGreenCal(bool blocking = true)
    
    Returns calibrated green reading from sensor.

    Calibrated Green Reading = (Green Raw - Green Low) / (Green High - Green Low)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns:

        * if ``setGreenRange()`` has been called, calibrated green reading from 0 to 1
        * -1 otherwise

.. function:: float readBlueNorm()
    
    Returns calibrated blue reading from sensor.

    Calibrated Blue Reading = (Blue Raw - Blue Low) / (Blue High - Blue Low)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns:

        * if ``setBlueRange()`` has been called, calibrated blue reading from 0 to 1
        * -1 otherwise

Reading HSV Values
"""""""""""""""""""

.. function:: float readHue()

    Returns the Hue component of the RGB readings when converted to the HSV colour space.

    :returns: Hue component of HSV reading (0-360deg)

.. function:: float readSaturation()

    Returns the Saturation component of the RGB readings when converted to the HSV colour space.

    :returns: Saturation component of HSV reading (0-1)

.. function:: float readValue()
    
    Returns the Value component of the RGB readings when converted to the HSV colour space.

    :returns: Value component of HSV reading (0-1)