``EVNEnvSensor``
===========================

This class provides the following features and functionalities for our Environment Sensor Standard Peripheral (BME280 IC):

    * Temperature, Humidity & Pressure Measurements
    * Altitude Calculation (based on pressure measurement & local sea-level pressure)
    * Non-Blocking & Blocking Read Functions

.. note:: This class does I2C port selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

Wiring (I2C)
------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
3V3   VIN         3.3V Power
GND   GND         Ground (0V)
SCL   SCL         I2C Serial Clock
SDA   SDA         I2C Serial Data
 --   CSB         Not Connected
 --   SDO         Not Connected
====  ==========  ===========

Constructor
-----------

.. class:: EVNEnvSensor(uint8_t port, sampling sampling_temp = ENV_SAMPLING_X1, sampling sampling_pres = ENV_SAMPLING_X1, sampling sampling_hum = ENV_SAMPLING_X1, filter filter = ENV_FILTER_OFF, standby standby = ENV_STANDBY_MS_0_5)

    :param port: I2C port the sensor is connected to (1-16)
    :param sampling_temp: Sampling rate for temperature. Defaults to ``ENV_SAMPLING_X1``
    :param sampling_pres: Sampling rate for pressure. Defaults to ``ENV_SAMPLING_X1``
    :param sampling_hum: Sampling rate for humidity. Defaults to ``ENV_SAMPLING_X1``
    :param filter: Filter rate applied to readings. Defaults to ``ENV_FILTER_OFF``
    :param standby: Standby time between end of reading and start of new reading (in ms). Defaults to ``ENV_STANDBY_MS_0_5``

Functions
---------

.. function:: bool begin()

    Initializes environment sensor. Call this function before using the other functions.
    
    :returns: Boolean indicating whether the sensor was successfully initialized. If ``false`` is returned, all other functions will return 0.

.. note::
    If ``begin()`` returns ``false``, try checking your connections and I2C port number!

.. function:: void setSeaLevelPressure(float pressure)

    Set sea-level pressure used to calculate altitude

    :param: pressure (in Pa)

.. function:: void setTemperatureOffset(float offset)

    Set offset applied to temperature readings. E.g. offset of 3 increases all temperature readings by 3 deg Celsius

    :param: offset (in deg C)
    
Reading Temperature
"""""""""""""""""""

.. function::   float readTemp(bool blocking = true)
                float readTemperature(bool blocking = true)
                float readTemperatureC(bool blocking = true)

    Read temperature (in Celsius)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    
    :returns: temperature (in deg C)
    
.. function::   float readTemperatureF(bool blocking = true)

    Read temperature (in Fahrenheit)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    
    :returns: temperature (in deg F)

Reading Humidity
"""""""""""""""""""

.. function::   float readHum(bool blocking = true)
                float readHumidity(bool blocking = true)
    
    Read humidity (from 0-100%)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    
    :returns: humidity (0-100)

Reading Pressure & Altitude
"""""""""""""""""""""""""""

.. function::   float readPres(bool blocking = true)
                float readPressure(bool blocking = true)

    Read pressure in Pascals

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    
    :returns: altitude (in Pa)

.. function::   float readAltitude(bool blocking = true)
                float readAltitudeMetres(bool blocking = true)

    Read altitude in metres

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    
    :returns: altitude (in m)


.. function::   float readAltitudeFeet(bool blocking = true)

    Read altitude in feet

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    
    :returns: altitude (in ft)

Advanced Sensor Settings
""""""""""""""""""""""""

.. function:: void setMode(bool enable)

    Sets mode to run sensor in

    :param enable: Whether measurement is enabled

.. function:: void setSamplingRate(sampling sampling_temp, sampling sampling_hum, sampling sampling_pres)
    
    Sets sampling rate for each of the 3 measurements. Increasing sampling rate lowers variance at the cost of response time. 
    
    Additionally, note that temperature measurement cannot be disabled.

    :param sampling_temp: Sampling rate for temperature
    :param sampling_pres: Sampling rate for pressure
    :param sampling_hum: Sampling rate for humidity

    * ``ENV_SAMPLING_OFF`` (disables measurement)
    * ``ENV_SAMPLING_X1`` (1)
    * ``ENV_SAMPLING_X2`` (2)
    * ``ENV_SAMPLING_X4`` (4)
    * ``ENV_SAMPLING_X8`` (8)
    * ``ENV_SAMPLING_X16`` (16)

.. function:: void setFilterRate(filter filter)

    Sets filter rate applied to pressure measurements. Increasing filter rate lowers variance at the cost of response time. 

    :param filter: Filter rate for pressure measurements

    * ``ENV_FILTER_OFF`` (filter disabled)
    * ``ENV_FILTER_X2`` (2)
    * ``ENV_FILTER_X4`` (4)
    * ``ENV_FILTER_X8`` (8)
    * ``ENV_FILTER_X16`` (16)

.. function:: void setStandbyTime(standby standby_time)

    Sets standby time between end of reading and start of new reading (in ms)

    :param standby_time: Standby time between readings

    * ``ENV_STANDBY_MS_0_5`` (0.5ms)
    * ``ENV_STANDBY_MS_10`` (10ms)
    * ``ENV_STANDBY_MS_20`` (20ms)
    * ``ENV_STANDBY_MS_62_5`` (62.5ms)
    * ``ENV_STANDBY_MS_125`` (125ms)
    * ``ENV_STANDBY_MS_250`` (250ms)
    * ``ENV_STANDBY_MS_500`` (500ms)
    * ``ENV_STANDBY_MS_1000`` (1000ms)

