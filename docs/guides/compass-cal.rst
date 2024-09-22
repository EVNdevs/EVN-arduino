Compass Calibration
==========================

If you have tried using the compass sensor, you may find that the readings are not great all the time. In fact, you may find that they are **bad** most of the time.
This is a guide to calibrate the compass sensor for much better readings.

Some Theory First!
------------------
While "compass" isn't a bad descriptor, a more technical term for this sensor is a 3-axis magnetometer.
It can measure magnetic field strength in the X, Y & Z axis (all perpendicular to each other).
The most common use for this is to measure the Earth's magnetic field (hence "compass"), but it can also be used for some other purposes (e.g. detecting the local presence of a magnet).

More importantly, the sensor captures **any** kind of nearby magnetic field, which results in interference. That is why calibration is needed.

Hard Iron Interference
""""""""""""""""""""""
Hard iron interference occurs when the sensor is placed close to any kind of permanent magnet (e.g. the ones in motors), as they produce their own magnetic field separate from the Earth's.

If placed too close, hard iron distortions can even overpower the magnitude of the Earth's magnetic field at the sensor's position, making it impossible to detect heading.
The solution to this is to move the sensor further away from the magnets by elevating it.

Soft Iron Interference
""""""""""""""""""""""
Soft iron interference occurs when the sensor is placed near ferromagnetic (attracted by magnets) metal. 

These metals stretch and distort the existing magnetic field (from the Earth or any other permanent magnets).

In most cases, the effects of soft iron interference are less pronounced than that of hard iron.

Limitations of Calibration
"""""""""""""""""""""""""""
Our calibration aims to remove the distortions caused by hard-iron and soft-iron sources. 

However, these distortions change when the distance and differences in orientation between the sensor and sources change. For example, 
the distortion created by a strong magnet 10cm away can differ greatly in magnitude from that of a strong magnet 20cm away.

So our calibration can only really compensate for **internal** sources of hard iron interference that always remain at the same position and orientation relative to the sensor. 
For instance, the permanent magnets in a robot's motors usually remain at the same distance and orientation relative to the sensor, so its interference can be accounted for.

But this isn't to say calibration is useless. Usually it can make the difference between "useful" and "unusable" readings. 
Just keep in mind that the more your robot's environment/location changes, the less valid your calibration values may be.

For a better explanation by people who know a lot more, check out this `webpage`_ by VectorNav. 

Calibration
------------

As long as you have enough readings of the robot in various orientations, one could plot and calculate the calibration values with a math program like MATLAB, or code.

However, what we will be doing is using a ready-made program called MotionCal made by Paul Stoffregen of `PJRC`_, creator of the Teensy microcontrollers. 

MotionCal will read the raw XYZ magnetometer readings and spit out the calibration values once enough readings have been collected.

1. Download MotionCal for your OS from `this page`_.
    
    You may have to right-click, and then select ``Save link as...`` to download the file.

2. Connect your Compass Sensor Standard Peripheral to an I2C port on EVN Alpha.

3. Upload the calibrateCompass.ino sketch from the EVN library examples to EVN Alpha.

    This sketch can be found in File > Examples > EVN > Others > Calibration > calibrateCompass.

    Remember to set the I2C port to the port your Compass Sensor is connected to!

    Once uploaded, Alpha should appear as a COM Port, and when it's selected Serial Monitor should depict the raw (uncalibrated) readings in the format below.

    .. code-block::

        Raw:0,0,0,0,0,0,312,120,313
        Raw:0,0,0,0,0,0,302,112,331
        Raw:0,0,0,0,0,0,294,123,342
        Raw:0,0,0,0,0,0,283,111,353

    Once you've checked that it's all working, close Serial Monitor to leave the COM port available for MotionCal to read from.

3. Open MotionCal. Select the COM Port of your EVN Alpha.

    .. image:: ../images/motioncal/motioncal1.png
    
    Once selected, MotionCal will begin plotting each reading as a red dot in a 3D space (X, Y and Z).
    
    .. image:: ../images/motioncal/motioncal2.png

4. Start rotating the robot. Ideally, there should be enough readings to cover the entire "sphere" of possible orientations.

    .. image:: ../images/motioncal/motioncal3.png

    As you collect more readings, the 4 error values at the bottom of the window will decrease. 
    When the errors are deemed low enough, the status circle on the left will turn green, and the Send Cal button becomes available (although we will not be using it).

    .. image:: ../images/motioncal/motioncal4.png

5. Note down the 12 values on the right of the screen from left-to-right, top-to-bottom, and place them in the EVNCompassSensor declaration.

    .. image:: ../images/motioncal/motioncal5.png

    Previously, our declaration may have looked like this:

    .. code-block:: c++

        EVNCompassSensor compass(1);

    But the 12 values will now be added to the declaration, as shown in the example below:

    .. code-block:: c++

        EVNCompassSensor compass(1, 1.93, 34.79, 19.87,
                                    1.027, 0.006, 0.008,
                                    0.006, 0.960, 0.013,
                                    0.008, 0.013, 1.014);

6. Test your new calibration!

    As a quick test, you can run the readCompass example sketch (File > Examples > EVN > 1. Basics > c) Sensors > readCompass), but with the new calibration values substituted in.

    When the sensor's heading (or yaw) is printed as a value from 0 to 360, it should now closely match the real-life heading of the sensor.

    But if you want a more thorough check, we can use the testCalibratedCompass sketch (File > Examples > EVN > Others > Calibration > testCalibratedCompass).

    It's exactly the same as calibrateCompass, but it sends calibrated readings instead of uncalibrated ones to MotionCal.

    When you've gathered enough data points, the 12 values should be very close to the values below (but small deviations are to be expected).

    .. code-block:: c++

        0 0 0   //hard iron offsets

        1 0 0   //soft iron offsets
        0 1 0
        0 0 1

    .. image:: ../images/motioncal/motioncal6.png


.. _this page: https://www.pjrc.com/store/prop_shield.html
.. _webpage: https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-hsicalibration
.. _pjrc: https://www.pjrc.com/