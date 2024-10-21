``EVNADC``
================

This class provides the following features and functionalities for our 4-Channel ADC Standard Peripheral (ADS1115 IC):

    * 16-bit precision Analog-to-Digital Converter (ADC)
    * ADC can be multiplexed to 4 Single-Ended Inputs or 2 Differential Inputs
    * Programmable Data Rates from 8 samples/second to 860 samples/second.

In short, this peripheral acts as a replacement for the built-in analog input pins one may find on other microcontrollers.

.. note:: This class does I2C port selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

Wiring (I2C)
------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
3V3   V           3.3V Power
GND   G           Ground (0V)
SCL   SCL         I2C Serial Clock
SDA   SDA         I2C Serial Data
 --   ADDR        Not Connected
 --   ALERT       Not Connected
 --   A0          Analog Input 0
 --   A1          Analog Input 1
 --   A2          Analog Input 2
 --   A3          Analog Input 3
====  ==========  ===========

The analog inputs connect to your devices with analog output e.g. analog light sensors or microphones.

Minimally, you will also need to connect ground (GND) to these analog devices as well. You may also need to connect power (3V3) to power the device.

The 4 analog input channels are not rated for voltages higher than the peripheral's supply voltage (usually 3.3V), so do not connect voltages higher than 3.3V to the inputs.

If you need voltage measurement ranging up to 5V, consider powering the peripheral with 5V instead, since the I2C ports are 5V-tolerant.

Constructor
-----------

.. class:: EVNADC(uint8_t port)

    :param port: I2C port the 4-channel ADC is connected to (1-16)

Functions
---------
.. function:: bool begin()

    Initializes 4-channel ADC. Call this function before using the other functions.

    :returns: Boolean indicating whether the 4-channel ADC was successfully initialized. If ``false`` is returned, all other functions will return 0.

Set Functions
"""""""""""""""

.. function:: void setDataRate(EVNADC::data_rate data_rate)

    :param data_rate: Data rate of ADC measurements
    
    * ``EVNADC::data_rate::SPS_8``
    * ``EVNADC::data_rate::SPS_16``
    * ``EVNADC::data_rate::SPS_32``
    * ``EVNADC::data_rate::SPS_64``
    * ``EVNADC::data_rate::SPS_128``
    * ``EVNADC::data_rate::SPS_250``
    * ``EVNADC::data_rate::SPS_475``
    * ``EVNADC::data_rate::SPS_860`` (default)

.. function:: void setRange(EVNADC::range range)

    :param range: Input voltage range of ADC measurements
    
    * ``EVNADC::range::MV_256``
    * ``EVNADC::range::MV_512``
    * ``EVNADC::range::MV_1024``
    * ``EVNADC::range::MV_2048``
    * ``EVNADC::range::MV_4096`` (default)
    * ``EVNADC::range::MV_6144``

Get Functions
"""""""""""""

.. function:: EVNADC::data_rate getDataRate()
    
    :returns: Input voltage range that ADC is set to

.. function:: EVNADC::range getRange()

    :returns: Input voltage range that ADC is set to

Single-Shot Measurement (Synchronous)
""""""""""""""""""""""""""""""""""""""

This kind of measurement is the same as the ``analogRead()`` functionality you get on Arduino boards.

The host requests the ADC to start a measurement and waits for the ADC to measure before obtaining and outputting the reading.

.. function:: float read(uint8_t pin)

    Reads analog voltage on an input pin

    :param pin: pin to read analog voltage from (0-7). When set from 0-3, the ADC will measure the voltage between pins 0-3 and GND. When set from 4-7, the ADC will measure the voltage between 2 pins:

    * "Pin" 4: Voltage between pin 0 and pin 1
    * "Pin" 5: Voltage between pin 0 and pin 3
    * "Pin" 6: Voltage between pin 1 and pin 3
    * "Pin" 7: Voltage between pin 2 and pin 3

    :returns: analog voltage measured in microvolts (one millionth of a volt)

Single-Shot Measurement (Asynchronous)
""""""""""""""""""""""""""""""""""""""""""

Asynchronous single-shot measurements allow you to request the ADC for a measurement, but continue to do other tasks until the measurement is ready.

Once ready, you can receive the reading like normal.

.. function:: bool request(uint8_t pin)

    Requests for analog voltage on an input pin

    :param pin: pin to read analog voltage from (0-7). When set from 0-3, the ADC will measure the voltage between pins 0-3 and GND. When set from 4-7, the ADC will measure the voltage between 2 pins:

    * "Pin" 4: Voltage between pin 0 and pin 1
    * "Pin" 5: Voltage between pin 0 and pin 3
    * "Pin" 6: Voltage between pin 1 and pin 3
    * "Pin" 7: Voltage between pin 2 and pin 3

.. function:: bool ready()

    :returns: Whether requested ADC measurement is ready. If no request has been made, it returns ``true``.

.. function:: float receive(bool blocking = true)

    Receive analog voltage measurement from ADC

    :param blocking: whether to wait until measurement is ready. Defaults to ``true``
    :returns: analog voltage measured in microvolts (one millionth of a volt)

Continuous Measurement
"""""""""""""""""""""""

The ADC can be set to run continuous measurements back-to-back, so users can enjoy asynchronous/synchronous measurement without needing to call ``request()``.

However, it can only be set to run on one pin at a time.

.. function:: void startContinuous(uint8_t pin)
    
    Call this function once to start continuous measurement on a pin.

    To end continuous measurement, call ``read()`` or ``request()`` to set the ADC back to single-measurement mode.

    :param pin: pin to read analog voltage from (0-7). When set from 0-3, the ADC will measure the voltage between pins 0-3 and GND. When set from 4-7, the ADC will measure the voltage between 2 pins:

    * "Pin" 4: Voltage between pin 0 and pin 1
    * "Pin" 5: Voltage between pin 0 and pin 3
    * "Pin" 6: Voltage between pin 1 and pin 3
    * "Pin" 7: Voltage between pin 2 and pin 3

.. function:: bool readyContinuous()

    :returns: Whether new ADC measurement in continuous mode is ready.

.. function:: float readContinuous(bool blocking = true)

    Returns analog voltage measurement from ADC (running in continuous mode)

    :param blocking: whether to wait until measurement is ready. Defaults to ``true``
    :returns: analog voltage measured in microvolts (one millionth of a volt)
