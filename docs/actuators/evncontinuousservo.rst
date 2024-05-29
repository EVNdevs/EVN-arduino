``EVNContinuousServo``
======================

EVNContinuousServo is a class for controlling 3-wire hobby (RC) **continuous-rotation** servo motors.

These motors offer control over the rotation speed of the servo shaft, rather than positional control.

However, the servo can be disabled using the user button similar to ``EVNMotor`` and ``EVNServo``.

The default settings are for the Continuous Rotation Servo Standard Peripherals we sell, the Geekservo Continuous Rotation Servo, but this library is compatible with any continuous rotation servo.

For positional control with the Fixed Servo Standard Peirpherals, look at `EVNServo`_.

.. _EVNServo: evnservo.html

.. note::

    By default, EVNContinuousServo objects will not move the servo until the user button is pressed. See `EVNAlpha`_'s docs for more info.
.. _EVNAlpha: ../evnalpha.html

.. warning::

    Geekservo servos can receive pulses from 500-2500us, but it is possible to damage certain servos by writing pulses outside their allowed ranges.
    If you are using a new servo, consider checking the servo instructions and specs to see if there is a rated pulse range given, and if there aren't any,
    try starting by setting min_pulse and max_pulse to 1000 and 2000 respectively.

Wiring (Servo)
--------------

====  ============   ===========
Host  Peripheral     Description
====  ============   ===========
SIG   SIG (Yellow)   Signal
5V    VCC (Red)      5V Power
GND   GND (Brown)    Ground (0V)
====  ============   ===========

Constructor
-----------

.. class:: EVNContinuousServo(uint8_t port, bool servo_dir = DIRECT, uint16_t min_pulse_us = 600, uint16_t max_pulse_us = 2400)
    
    :param port: Port the continuous rotation servo is connected to (1 - 4)
    :param servo_dir: Direction of rotation of motor shaft. On Geekservo, ``DIRECT`` is clockwise. Defaults to ``DIRECT``
    :param min_pulse_us: Minimum pulse time sent to the servo motor (in microseconds). Defaults to 600
    :param max_pulse_us: Maximum pulse time sent to the servo motor (in microseconds). Defaults to 2400
    
Example Usage:

.. code-block:: cpp

    EVNContinuousServo servo(1);

    EVNContinuousServo servo2(2, DIRECT, 600, 2400);

Functions
---------

.. function:: void begin()

    Initializes continuous rotation servo object. Call this function before calling the other EVNContinuousServo functions.

.. note::
    For best performance, run this on the 2nd core using ``void setup1()``

Example Program:

.. code-block:: cpp

    EVNContinuousServo servo(1);

    void setup1()   //call on setup1() for best performance!
    {
        servo.begin();
    }

Using Continuous Servos
""""""""""""""""""""""""

.. function::   void write(float duty_cycle)
                void writeDutyCycle(float duty_cycle)

    Runs continuous rotation servo at given duty cycle (-100% - 100%)

    :param duty_cycle: Duty cycle to run servo at (-1 to 1).


.. function:: void writeMicroseconds(float pulse_us)

    Sends pulse of given length to continuous rotation servo

    :param pulse_us: Pulse time to transmit to continuous rotation servo (in microseconds) from 200us to 2800us
