``EVNServo``
============

EVNServo is a class for controlling 3-wire hobby (RC) servo motors. 
However, the servo can be disabled using the user button similar to ``EVNMotor``.
Additionally, EVNServo uses timer-generated interrupts to write to the servo motor, allowing for smooth sweeping motion at various speeds without the need for ``delay()``. 
The default settings are for the Fixed Servo Standard Peripherals we sell, the Geekservo 270deg Servo, but this library is compatible with any fixed range servo.

.. note::

    By default, EVNServo objects will not move the servo's position until the user button is pressed. See EVNAlpha's docs for more info.

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

.. class:: EVNServo(uint8_t port, bool servo_dir = DIRECT, uint16_t range = 270, float start_angle = 135, uint16_t min_pulse_us = 600, uint16_t max_pulse_us = 2400, float max_dps = 500)
    
    :param port: Port the servo is connected to (1 - 4)
    :param servo_dir: Direction of rotation of motor shaft. On Geekservo, ``DIRECT`` is clockwise. Defaults to ``DIRECT``
    :param range: Angular range of servo (in degrees). Defaults to 270
    :param start_angle: Starting angle of motor shaft, when servo is initialised. Defaults to 135
    :param min_pulse_us: Minimum pulse time sent to the servo motor (in microseconds). Defaults to 600
    :param max_pulse_us: Maximum pulse time sent to the servo motor (in microseconds). Defaults to 2400
    :param max_dps: Maximum angular rotation of servo shaft (in degrees per second). Defaults to 500


.. note:: For continuous servos, the ``range``, ``start_angle`` and ``max_dps`` fields are ignored.
    
Example Usage:

.. code-block:: cpp

    EVNServo servo(1);

    //Geekservo 2kg 360Deg Servo
    EVNServo servo2(2, 0, DIRECT, 360, 600, 2400, 428);

Functions
---------

.. function:: void begin()

    Initializes servo object. Call this function before calling the other EVNServo functions.

.. note::
    For best performance, run this on the 2nd core using ``void setup1()``

Example Program:

.. code-block:: cpp

    EVNServo servo(1);

    void setup1()   //call on setup1() for best performance!
    {
        servo.begin();
    }

Using Fixed Servos
""""""""""""""""""

.. function:: float getMaxDPS()

    :returns: Maximum angular rotation of servo shaft (in degrees per second).

.. function:: uint16_t getRange()

    :returns: Angular range of servo (in degrees).

.. function::   void write(float angle, float wait_time_ms, float dps)
                void writeAngle(float angle, float wait_time_ms, float dps)

    Rotate motor shaft to given angle.

    :param angle: Position to run servo shaft to (in degrees)
    :param wait_time_ms: Time to wait before continuing the program (in milliseconds). Same effect as ``delay()``, but terminates when servos are disabled.
    :param dps: Speed to run servo at (in degrees per second), from 0 to **max_range**. When dps is 0, servo runs at max speed. Defaults to 0.


.. function:: void writeMicroseconds(float pulse_us, float wait_time_ms)

    Sends pulse of given length to servo.

    :param pulse_us: Pulse time to transmit to servo (in microseconds) from 200us to 2800us
    :param wait_time_ms: Time to wait before continuing the program (in milliseconds). Same effect as ``delay()``, but terminates when servos are disabled.
