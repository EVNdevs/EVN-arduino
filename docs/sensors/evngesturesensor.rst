``EVNGestureSensor``
====================

This class provides the following features and functionalities for our Gesture Sensor Standard Peripheral (APDS9960 IC):

    * Direction Gesture Detection (Up, Down, Left, Right)
    * Proximity Detection
    * Passive Red, Green, Blue (RGB) & Clear Light Measurements (no onboard LEDs)

.. note:: This class does I2C port selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

Wiring (I2C)
------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
 --   VL          Not Connected
GND   GND         Ground (0V)
3V3   VCC         3.3V Power
SDA   SDA         I2C Serial Data
SCL   SCL         I2C Serial Clock
 --   INT         Not Connected
====  ==========  ===========

Usage
--------------

While this sensor has 3 functions (Gesture, Colour, Proximity), not all of them cannot be used simultaneously.

* When Gesture Mode is enabled, the Colour and Proximity readings must be disabled.
* When Colour and Proximity Modes are enabled, Gesture Mode must be disabled.

For convenience, the library does the following:

* ``setGestureMode(true)`` automatically **disables** Proximity and Colour Mode
* ``setColourMode(true)`` automatically **disables** Gesture Mode
* ``setProximityMode(true)`` automatically **disables** Gesture Mode
* By default, Gesture Mode is enabled, with Colour Mode and Proximity Mode disabled

.. note::

    When Gesture Mode is disabled, the ``gestureDetected()`` and non-blocking ``readGesture()`` will be disabled. 
    However, a blocking ``readGesture()`` (``readGesture(true)``) will still be fully functional.

Constructor
-----------

.. class:: EVNGestureSensor(uint8_t port)

    :param port: I2C port the sensor is connected to (1-16)

Functions
---------

.. function:: bool begin()

    Initializes gesture sensor. Call this function before using the other functions.

    By default, Gesture Mode is enabled, with Colour Mode and Proximity Mode disabled.

    :returns: Boolean indicating whether the sensor was successfully initialized. If ``false`` is returned, all other functions will return 0.

Gesture Detection
"""""""""""""""""
Gestures are detected by calling the ``readGesture()``, ``readGestureUpDown()`` or ``readGestureLeftRight()`` functions.

The functions return numbers which correspond to the 4 directional gestures of the sensor, with ``GESTURE_NONE`` representing invalid gestures/no gestures.

=================  ======
Definition         Number
=================  ======
``GESTURE_NONE``   1
``GESTURE_UP``     2
``GESTURE_DOWN``   3
``GESTURE_LEFT``   4
``GESTURE_RIGHT``  5
=================  ======

The function outputs can be evaluated against numbers or their written versions (in C, they're called macros).

.. code-block:: c

    void loop() {
        if (readGesture() == 0)
        {
            //this works
        }

        if (readGesture() == GESTURE_NONE)
        {
            //this works too
        }
    }

.. function:: uint8_t readGesture(bool blocking = false, uint32_t timeout_ms = 5000)

    The function starts by checking if the start of a gesture has been detected by the sensor.

    If ``blocking`` is ``false`` and no gesture start (i.e. no movement) has been detected, ``GESTURE_NONE`` will be returned.
    But if ``blocking`` is ``true``, the function will continue waiting for a gesture start until its runtime has exceeded ``timeout_ms``.

    Once a gesture start is obtained, the function will continuously read the sensor data until the gesture ends or is deemed invalid.
    For example, if a human hand enters the sensor's view but hovers around the sensor without exiting, it is invalid.

    The gesture is then returned. Invalid gestures are returned as ``GESTURE_NONE``.

    :param blocking: Boolean indicating whether to wait for new gesture to begin. Defaults to ``false``
    :param timeout_ms: Time the sensor should wait for a new gesture before returning when ``blocking`` is ``true``  (in milliseconds). Defaults to 5000
    :returns: Numerical representation of gesture

.. function:: uint8_t readGestureUpDown(bool blocking = false, uint32_t timeout_ms = 5000)

    Same as ``readGesture``, but only returns ``GESTURE_NONE``, ``GESTURE_UP`` or ``GESTURE_DOWN``. Ensures that diagonal gestures will be not returned as ``GESTURE_LEFT`` or ``GESTURE_RIGHT``.

    :param blocking: Boolean indicating whether to wait for new gesture to begin. Defaults to ``false``
    :param timeout_ms: Time the sensor should wait for a new gesture before returning when ``blocking`` is ``true``  (in milliseconds). Defaults to 5000
    :returns: Numerical representation of gesture (``GESTURE_NONE``, ``GESTURE_UP`` or ``GESTURE_DOWN``)

.. function:: uint8_t readGestureLeftRight(bool blocking = false, uint32_t timeout_ms = 5000)

    Same as ``readGesture``, but only returns ``GESTURE_NONE``, ``GESTURE_LEFT`` or ``GESTURE_RIGHT``. Ensures that diagonal gestures will be not returned as ``GESTURE_UP`` or ``GESTURE_DOWN``.

    :param blocking: Boolean indicating whether sensor should wait for new gesture to begin. Defaults to false
    :param timeout_ms: Time the sensor should wait for a new gesture before returning when ``blocking`` is ``true``  (in milliseconds). Defaults to 5000
    :returns: Numerical representation of gesture (``GESTURE_NONE``, ``GESTURE_LEFT`` or ``GESTURE_RIGHT``)

.. function:: bool gestureDetected()

    :returns: Boolean indicating whether the start of a gesture has been detected by the sensor

Proximity Detection
"""""""""""""""""""

.. function:: uint8_t readProximity(bool blocking = true)

    Returns proximity reading of any object within sensor's view to the sensor (from 0-255). A higher value indicates that the object is closer to the sensor, as readings are
    based on the intensity of infrared light emitted by the sensor being reflected back into the sensor.

    :returns: Proximity reading (0-255)

RGBC Colour Detection
"""""""""""""""""""""

.. function:: uint16_t readClear(bool blocking = true)

    Returns clear light reading from sensor.

    :returns: Clear reading

.. function:: uint16_t readRed(bool blocking = true)

    Returns red light reading from sensor.

    :returns: Red reading

.. function:: uint16_t readGreen(bool blocking = true)

    Returns green light reading from sensor.

    :returns: Green reading

.. function:: uint16_t readBlue(bool blocking = true)

    Returns clear blue reading from sensor.

    :returns: Blue reading

Sensor Settings
"""""""""""""""

.. function:: void setPower(bool enable)
.. function:: void setLEDBoost(led_boost boost)
.. function:: void setWait(bool enable)

.. function:: void setGestureMode(bool enable)

    Enable/disable Gesture Mode.
    If ``enable`` is ``true``, this will disable Colour Mode and Proximity Mode as well.

    :param enable: Whether Gesture Mode should be enabled

.. function:: bool getGestureMode()

    :returns: Whether Gesture Mode is enabled

.. function:: void setGestureLED(led_curr current)
.. function:: void setGestureGain(gesture_gain gain)
.. function:: void setGesturePulseCount(uint8_t pulse_count)
.. function:: void setGesturePulseLength(pulse_len pulse_length)
.. function:: void setGestureFIFOThreshold(gesture_fifo threshold)
.. function:: void setGestureEntryThreshold(uint8_t threshold)
.. function:: void setGestureExitThreshold(uint8_t threshold)
.. function:: void setGestureDimensions(gesture_dims dims)

.. function:: void setProximityMode(bool enable)

    Enable/disable Proximity Mode.
    If ``enable`` is ``true``, this will disable Gesture Mode as well.

    :param enable: Whether Proximity Mode should be enabled

.. function:: bool getProximityMode()

    :returns: Whether Proximity Mode is enabled

.. function:: void setProximityGain(proximity_gain gain)
.. function:: void setProximityLED(led_curr current)
.. function:: void setProximityPulseCount(uint8_t pulse_count)
.. function:: void setProximityPulseLength(pulse_len pulse_length)
    
.. function:: void setColourMode(bool enable)

    Enable/disable Colour Mode.
    If ``enable`` is ``true``, this will disable Gesture Mode as well.

    :param enable: Whether Colour Mode should be enabled

.. function:: bool getColourMode()

    :returns: Whether Colour Mode is enabled

.. function:: void setColourGain(colour_gain gain)
.. function:: void setColourIntegrationCycles(uint16_t int_cycles)
.. function:: uint16_t getColourIntegrationCycles()