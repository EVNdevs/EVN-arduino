``EVNContinuousServo``
======================

EVNContinuousServo is a class for controlling 3-wire hobby (RC) **continuous-rotation** servo motors.

These motors offer control over the rotation speed of the servo shaft, rather than positional control.

However, the servo can be disabled using the user button similar to ``EVNMotor`` and ``EVNServo``.

The default settings are for the Continuous Rotation Servo Standard Peripherals we sell, the Geekservo Continuous Rotation Servo, but this library is compatible with any continuous rotation servo.

For positional control with the Fixed Servo Standard Peirpherals, look at `EVNServo`_.

.. note:: Each instance of EVNContinuousServo (4 max) consumes 1 of the RP2040's 8 PIO state machines, so keep this in mind if you are using PIO for your own purposes.

.. note:: EVNContinuousServo consumes some of the RP2040's spinlock peripherals. See this `page`_ for more info.

.. _page: ../getting-started/hardware-overview.html

.. _EVNServo: evnservo.html
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
    
    .. code-block:: cpp

        EVNContinuousServo cservo(1);

        EVNContinuousServo cservo(2, DIRECT, 600, 2400);

Functions
---------

.. function:: void begin()

    Initializes continuous rotation servo. Call this function before calling the other EVNContinuousServo functions.

    .. code-block:: cpp

        EVNContinuousServo cservo(1);

        void setup1()   //call on setup1() for best performance!
        {
            cservo.begin();
        }

.. note::
    For best performance, run this on the 2nd core using ``void setup1()``

.. function::   void write(float duty_cycle_pct)

    Runs continuous rotation servo at given duty cycle in %(-100 to 100)

    :param duty_cycle_pct: Duty cycle to run servo at (Number from -100 to 100).

    .. code-block:: cpp

        //write servo to run at 80% duty cycle
        cservo.write(80);

.. function:: void writeMicroseconds(uint16_t pulse_us)

    Sends pulse of given length to continuous rotation servo

    :param pulse_us: Pulse time to transmit to continuous rotation servo (in microseconds) from 200us to 2800us

    .. code-block:: cpp

        //write 1500us pulse to continuous rotation servo
        cservo.writeMicroseconds(1500);

.. function:: void setMode(bool enable)

    Enable/disable continuous servo. When disabled, the PIO state machine consumed by the EVNContinuousServo object is released.

    :param enable: Whether to enable continuous servo operation