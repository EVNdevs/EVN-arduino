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

.. class:: EVNColourSensor(uint8_t port, uint8_t integration_cycles = 1, gain gain = COLOUR_GAIN_X16)

    The listed default settings for the sensor are listed here. Refer to ``setGain()`` & ``setIntegrationTime()`` for more information.

    :param port: I2C port the sensor is connected to (1-16)
    :param integration_cycles: Number of 2.4ms integration cycle for one reading. Defaults to 1
    :param gain: Gain applied to readings. Defaults to ``COLOUR_GAIN_X16``

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

        * ``COLOUR_GAIN_X1`` -- 1x gain
        * ``COLOUR_GAIN_X4`` -- 4x gain
        * ``COLOUR_GAIN_X16`` -- 16x gain
        * ``COLOUR_GAIN_X60`` -- 60x gain

.. function:: void setIntegrationCycles(uint8_t integration_cycles)

    Sets the number of 2.4ms integration cycles used for 1 reading.

    Integration Time = 2.4ms * Integration Cycles

    :param integration_cycles: Number of 2.4ms integration cycles for 1 reading (1-255)

Reading Raw RGBC Values
"""""""""""""""""""""""

.. function:: uint16_t read(uint8_t component, bool blocking = true)

    Returns raw reading for given component (Clear, Red, Green or Blue).

    :param component: RGBC Component to be returned

        * ``CLEAR``
        * ``RED``
        * ``GREEN``
        * ``BLUE``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: raw reading

.. function::   uint16_t readClear(bool blocking = true)

    Same as ``read(CLEAR, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: raw Clear reading

.. function:: uint16_t readRed(bool blocking = true)

    Same as ``read(RED, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: raw Red reading

.. function:: uint16_t readGreen(bool blocking = true)

    Same as ``read(GREEN, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: raw Green reading

.. function:: uint16_t readBlue(bool blocking = true)

    Same as ``read(BLUE, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: raw Blue reading

.. function:: float readPct(bool blocking = true)

    Returns raw reading for given component as a percentage (0 - 100).

    :param component: RGBC Component to be returned

        * ``CLEAR``
        * ``RED``
        * ``GREEN``
        * ``BLUE``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: raw reading in % (0 - 100)

.. function:: float readClearPct(bool blocking = true)

    Same as ``readPct(CLEAR, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: raw Clear reading in % (0 - 100)

.. function:: float readRedPct(bool blocking = true)

    Same as ``readPct(RED, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: raw Red reading in % (0 - 100)

.. function:: float readGreenPct(bool blocking = true)

    Same as ``readPct(GREEN, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: raw Green reading in % (0 - 100)

.. function:: float readBluePct(bool blocking = true)

    Same as ``readPct(BLUE, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: raw Blue reading in % (0 - 100)

Reading Normalised RGBC Values
"""""""""""""""""""""""""""""""
When you read the raw RGBC values, you may not have a reading of 0 on black surfaces or a maximum reading on white.

Instead, the readings will usually range from a low (but non-zero) to a high (but non-max) value.

Normalisation is the process of converting raw readings such that they range from 0 to 1 instead.

Normalised Reading = (Raw Reading - Low) / (High - Low)

Before reading normalised values, you first need to call the ``setRange()`` function to set the low and high values for a given colour channel.

.. function:: void setRange(uint8_t component, uint16_t low, uint16_t high)

    Set the range of raw values for given colour component.

    If this function is not called for a given colour component, ``readNorm()`` for that colour component will return 0.

    :param component: RGBC Component to be set range for

        * ``CLEAR``
        * ``RED``
        * ``GREEN``
        * ``BLUE``

    :param low: lower bound of readings for colour component
    :param high: upper bound of readings for colour component

.. function:: void setClearRange(uint16_t low, uint16_t high)
    
    Same as ``setRange(CLEAR, low, high)``

    :param low: lower bound of Clear readings
    :param high: upper bound of Clear readings

.. function:: void setRedRange(uint16_t low, uint16_t high)
    
    Same as ``setRange(RED, low, high)``

    :param low: lower bound of Red readings
    :param high: upper bound of Red readings

.. function:: void setGreenRange(uint16_t low, uint16_t high)
    
    Same as ``setRange(GREEN, low, high)``

    :param low: lower bound of Green readings
    :param high: upper bound of Green readings

.. function:: void setBlueRange(uint16_t low, uint16_t high)
    
    Same as ``setRange(BLUE, low, high)``

    :param low: lower bound of Blue readings
    :param high: upper bound of Blue readings

After calling these functions, you can use the ``readNorm()`` function to read normalised readings.

.. function:: float readNorm(uint8_t component, bool blocking = true)

    Returns normalised reading for given colour component in % (0 - 100)

    :param component: RGBC Component to be returned

        * ``CLEAR``
        * ``RED``
        * ``GREEN``
        * ``BLUE``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: normalised reading in % (0 - 100)

.. function:: float readClearNorm(bool blocking = true)

    Same as ``readNorm(CLEAR, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: normalised Clear reading in % (0 - 100)

.. function:: float readRedNorm(bool blocking = true)
    
    Same as ``readNorm(RED, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: normalised Red reading in % (0 - 100)

.. function:: float readGreenNorm(bool blocking = true)
    
    Same as ``readNorm(GREEN, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: normalised Green reading in % (0 - 100)

.. function:: float readBlueNorm(bool blocking = true)
    
    Same as ``readNorm(BLUE, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: normalised Blue reading in % (0 - 100)

Reading HSV Values
"""""""""""""""""""

By default, raw readings are used for HSV Colourspace conversion.

One can set HSV conversion to use normalised readings using ``useNormForHSV()``. 

.. function:: void useNormForHSV(bool enable)

    :param enable: Whether to use normalised readings for HSV conversion

.. function:: float readHSV(uint8_t component, bool blocking = true)

    Returns given component of reading when converted to HSV (Hue, Saturation or Value)

    :param component: HSV Component to be returned

        * ``HUE``
        * ``SAT``
        * ``VAL``
    
    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns:

        * ``HUE``: Hue component of reading in degrees (0-360)
        * ``SAT``: Saturation component of reading in % (0-100)
        * ``VAL``: Value component of reading in % (0-100)

.. function:: float readHue(bool blocking = true)

    Same as ``readHSV(HUE, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: Hue component of reading in degrees (0-360)

.. function:: float readSaturation(bool blocking = true)

    Same as ``readHSV(SAT, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: Saturation component of reading in % (0-100)

.. function:: float readValue(bool blocking = true)
    
    Same as ``readHSV(VAL, blocking)``

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: Value component of reading in % (0-100)