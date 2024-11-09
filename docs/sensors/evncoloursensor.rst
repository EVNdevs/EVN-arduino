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

Reading Normalised RGBC Values
"""""""""""""""""""""""""""""""
When you read the raw RGBC values, you may not have a reading of 0 on black surfaces or a maximum reading on white.

Instead, the readings will usually range from a low (but non-zero) to a high (but non-max) value.

Normalisation is the process of converting raw readings such that they range from 0 to 1 instead.

Normalised Reading = (Raw Reading - Low) / (High - Low)

Before reading normalised values, you need to call the ``setXXrange()`` function to set the low and high values for a given colour channel first.

.. function:: void setClearRange(uint16_t low, uint16_t high)
    
    Sets the range of possible clear values for raw readings

    If this function is not called, ``readClearNorm()`` returns 0

    :param low: lower bound of readings for Clear channel

    :param high: upper bound of readings for Clear channel

.. function:: void setRedRange(uint16_t low, uint16_t high)
    
    Sets the range of possible red values for raw readings

    If this function is not called, ``readRedNorm()`` returns 0

    :param low: lower bound of readings for Red channel

    :param high: upper bound of readings for Red channel

.. function:: void setGreenRange(uint16_t low, uint16_t high)
    
    Sets the range of possible green values for raw readings

    If this function is not called, ``readGreenNorm()`` returns 0

    :param low: lower bound of readings for Green channel

    :param high: upper bound of readings for Green channel

.. function:: void setBlueRange(uint16_t low, uint16_t high)
    
    Sets the range of possible blue values for raw readings

    If this function is not called, ``readBlueNorm()`` returns 0

    :param low: lower bound of readings for Blue channel

    :param high: upper bound of readings for Blue channel

After calling these functions, you can use the ``readXXNorm()`` functions to read normalised readings

.. function:: float readClearNorm(bool blocking = true)

    Returns normalised clear reading from sensor.

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns:

        * if ``setClearRange()`` has been called, normalised clear reading from 0 to 1
        * -1 otherwise

.. function:: float readRedNorm(bool blocking = true)
    
    Returns normalised red reading from sensor.

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns:

        * if ``setRedRange()`` has been called, normalised red reading from 0 to 1
        * -1 otherwise

.. function:: float readGreenNorm(bool blocking = true)
    
    Returns normalised green reading from sensor.

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns:

        * if ``setGreenRange()`` has been called, normalised green reading from 0 to 1
        * -1 otherwise

.. function:: float readBlueNorm()
    
    Returns normalised blue reading from sensor.

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns:

        * if ``setBlueRange()`` has been called, normalised blue reading from 0 to 1
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