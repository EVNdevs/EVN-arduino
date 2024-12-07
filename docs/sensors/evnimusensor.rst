``EVNIMUSensor``
================

This class provides the following features and functionalities for our 6DoF IMU Sensor Standard Peripheral (MPU6500 IC):

    * 3-Axis Accelerometer Measurements
    * 3-Axis Gyroscope Measurements

.. note:: This class does I2C port selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

Wiring (I2C)
------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
 --   FSYNC       Not Connected
 --   NCS         Not Connected
 --   INT         Not Connected
 --   AD0/SDO     Not Connected
 --   EDA         Not Connected
 --   ECL         Not Connected
SDA   SDA/SDI     I2C Serial Data
SCL   SCL/SCLK    I2C Serial Clock
GND   GND         Ground (0V)
3V3   VCC         3.3V Power
====  ==========  ===========

Constructor
-----------

.. class:: EVNIMUSensor(uint8_t port, data_rate data_rate = IMU_HZ_92, accel_range accel_range = IMU_ACCEL_G_4, gyro_range gyro_range = IMU_GYRO_DPS_500, float gx_offset = 0, float gy_offset = 0, float gz_offset = 0, float ax_low = 0, float ax_high = 0, float ay_low = 0, float ay_high = 0, float az_low = 0, float az_high = 0)

    :param port: I2C port the sensor is connected to (1-16)
    :param data_rate: Data rate of gyroscope and accelerometer measurements. Defaults to ``IMU_HZ_92``
    :param accel_range: Range of accelerometer measurements. Defaults to ``IMU_ACCEL_G_4``
    :param gyro_range: Range of gyroscope measurements. Defaults to ``IMU_GYRO_DPS_500``
    :param gx_offset: Gyroscope X-axis offset. Defaults to 0
    :param gy_offset: Gyroscope Y-axis offset. Defaults to 0
    :param gz_offset: Gyroscope Z-axis offset. Defaults to 0
    :param ax_low: Accelerometer X-axis low value. Defaults to 0
    :param ax_high: Accelerometer X-axis high value. Defaults to 0
    :param ay_low: Accelerometer Y-axis low value. Defaults to 0
    :param ay_high: Accelerometer Y-axis high value. Defaults to 0
    :param az_low: Accelerometer Z-axis low value. Defaults to 0
    :param az_high: Accelerometer Z-axis high value. Defaults to 0

Functions
---------

.. function:: bool begin(bool calibrate_gyro = true)

    Initializes IMU sensor. Call this function before using the other functions.

    :param calibrate_gyro: If ``true``, runs gyroscope calibration for 3 seconds. During this period, the robot should be at rest and not moving,
    :returns: Boolean indicating whether the sensor was successfully initialized. If ``false`` is returned, all other functions will return 0.

Accelerometer Measurements
""""""""""""""""""""""""""

.. function:: float readAccelX(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw X-axis accelerometer reading (in g)

.. function:: float readAccelY(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw Y-axis accelerometer reading (in g)

.. function:: float readAccelZ(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw Z-axis accelerometer reading (in g)

Gyroscope Measurements
""""""""""""""""""""""

.. function:: float readGyroX(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw X-axis gyroscope reading (in degrees per second)

.. function:: float readGyroY(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw Y-axis gyroscope reading (in degrees per second)

.. function:: float readGyroZ(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw Z-axis gyroscope reading (in degrees per second)

Fused Measurements
""""""""""""""""""
Gyroscope measurements are susceptible to drift. Gyroscope measure rate of rotation along each axis (X, Y & Z). This means that when perfectly stationary, a perfect gyroscope should return 0 along all axes. 

However, this perfect gyroscope does not exist. All gyroscopes are susceptible to drift in their readings. Even if a gyroscope is initially calibrated to return 0 when perfectly stationary,

the readings will drift over time based on various factors such as surrounding temperature, which makes it hard to correct for or even predict.

One solution to this is to fuse gyroscope readings with accelerometer and magnetometer readings. When well calibrated, accelerometer readings can compensate for pitch/roll drift, while magnetometer/compass readings can compensate for yaw drift.

There are many fusion algorithms out there, but we are using **Madgwick** fusion for this library. 
It is a stretch to say fusion makes these problems go away entirely (in the end, the fusion algorithm's output can only be as good as the data it receives), but it can make the difference between usable and unusable.

Additionally, there is a caveat: in order to obtain fused measurements, ``update()`` needs to be called in your loop at a frequency at least higher than the update rate of the IMU.

.. function::   void update()

    To use fused measurements, this function needs to be called at a rate higher than the update rate of your IMU Sensor Standard Peripheral (and the faster, the better).

    .. code-block:: cpp
        
        imu.update()

.. function::   float readYaw(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: Yaw orientation in degrees

.. function:: float readYawRadians(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: Yaw orientation in radians

.. function:: float readPitch(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: Pitch orientation in degrees

.. function:: float readPitchRadians(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: Pitch orientation in radians
    
.. function:: float readRoll(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: Roll orientation in degrees

.. function:: float readRollRadians(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``
    :returns: Roll orientation in radians

.. function:: void linkCompass(EVNCompassSensor* compass)

    Link compass to EVNIMUSensor. Once linked, ``update()`` will fuse all 3 sensor readings together. 
    
    The primary benefit of adding compass readings to sensor fusion is to compensate for yaw/heading drift in the gyroscope.

    :param compass: Pointer to EVNCompassSensor object (e.g. ``&compass``, where ``compass`` refers to an ``EVNCompassSensor`` object declared in the code)

    .. code-block:: cpp

        imu.linkCompass(&compass);

Sensor Settings
"""""""""""""""

The accelerometer and gyroscope measure along 3 different axes (X, Y and Z). This image depicts the 3 axes of the sensor.
As a quick reference, the sensor PCB has markings for the X and Y axis.
By default, the X axis is set as the axis passing through the front of the robot, and the Z axis as the axis passing through the top of the robot.

However, the IMU Sensor Standard Peripheral can be mounted in many orientations, hence the functions below can be used to set the correct axes.

.. function:: void setTopAxis(uint8_t axis)

    :param axis: Sensor axis that passes through the top of the robot (options shown below)

    * ``AXIS_X``
    * ``AXIS_Y``
    * ``AXIS_Z``

.. function:: void setFrontAxis(uint8_t axis)

    :param axis: Sensor axis that passes through the front of the robot (options shown below)

    * ``AXIS_X``
    * ``AXIS_Y``
    * ``AXIS_Z``

.. function:: void setAccelRange(accel_range range)

    :param range: Range of accelerometer measurements

    * ``IMU_ACCEL_G_2`` (+-2g)
    * ``IMU_ACCEL_G_4`` (+-4g)
    * ``IMU_ACCEL_G_8`` (+-8g)
    * ``IMU_ACCEL_G_16`` (+-16g)

.. function:: void setGyroRange(gyro_range range)

    :param range: Range of gyroscope measurements

    * ``IMU_GYRO_DPS_250`` (+-250DPS)
    * ``IMU_GYRO_DPS_500`` (+-500DPS)
    * ``IMU_GYRO_DPS_1000`` (+-1000DPS)
    * ``IMU_GYRO_DPS_2000`` (+-2000DPS)

.. function:: void setDataRate(data_rate data_rate)

    :param data_rate: Data rate of gyroscope and accelerometer measurements

    * ``IMU_HZ_5`` (5 Hz)
    * ``IMU_HZ_10`` (10 Hz)
    * ``IMU_HZ_20`` (20 Hz)
    * ``IMU_HZ_41`` (41 Hz)
    * ``IMU_HZ_92`` (92 Hz)
    * ``IMU_HZ_184`` (184 Hz)