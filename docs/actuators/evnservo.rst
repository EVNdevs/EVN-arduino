``EVNServo``
============

EVNServo is a class for controlling 3-wire hobby (RC) servo motors. 
However, the servo can be disabled using the user button similar to `EVNMotor`_.
Additionally, EVNServo uses timer-generated interrupts to write to the servo motor, allowing for smooth sweeping motion at various speeds without the need for ``delay()``. 
The default settings are for the Fixed Servo Standard Peripherals we sell, the Geekservo 270deg Servo, but this library is compatible with any fixed range servo.

.. _EVNMotor: evnmotor.html

.. note:: Each instance of EVNServo (4 max) consumes 1 of the RP2040's 8 PIO state machines, so keep this in mind if you are using PIO for your own purposes.

.. note:: EVNServo consumes some of the RP2040's spinlock peripherals. See this `page`_ for more info.

.. _page: ../getting-started/hardware-overview.html

.. warning::

    Geekservo servos can receive pulses from 500-2500us, but it is possible to damage certain servos by writing pulses outside their allowed ranges.
    If you are using a new servo, consider checking the servo instructions and specs to see if there is a rated pulse range given, and if there aren't any,
    try starting by setting min_pulse and max_pulse to 1000 and 2000 respectively.

.. _EVNAlpha: evnalpha.html

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

.. class:: EVNServo(uint8_t port, bool servo_dir = DIRECT, uint16_t range = 270, float start_position = 135, uint16_t min_pulse_us = 600, uint16_t max_pulse_us = 2400, float max_dps = 500)
    
    :param port: Port the servo is connected to (1 - 4)
    :param servo_dir: Direction of rotation of motor shaft. On Geekservo, ``DIRECT`` is clockwise. Defaults to ``DIRECT``
    :param range: Angular range of servo (in degrees). Defaults to 270
    :param start_position: Starting position of motor shaft when servo is initialised (in deg). Defaults to 135
    :param min_pulse_us: Minimum pulse time sent to the servo motor (in microseconds). Defaults to 600
    :param max_pulse_us: Maximum pulse time sent to the servo motor (in microseconds). Defaults to 2400
    :param max_dps: Maximum angular rotation of servo shaft (in degrees per second). Defaults to 500
    
Example Usage:

.. code-block:: cpp

    EVNServo servo(1);

    //Geekservo 2kg 360Deg Servo
    EVNServo servo2(2, 0, DIRECT, 360, 600, 2400, 428);

Functions
---------

.. function:: void begin()

    Initializes servo. Call this function before calling the other EVNServo functions.

    .. code-block:: cpp
        
        EVNServo servo(1);

        void setup1()   //call on setup1() for best performance!
        {
            servo.begin();
        }

.. note::
    For best performance, run this on the 2nd core using ``void setup1()``

.. function:: float getMaxDPS()

    :returns: Maximum angular rotation of servo shaft (in degrees per second).

    .. code-block:: cpp

        float max_dps = servo.getMaxDPS();

.. function:: uint16_t getRange()

    :returns: Angular range of servo (in degrees).

    .. code-block:: cpp

        int range = servo.getRange();

.. function::   void write(float position, uint16_t wait_time_ms, float dps)

    Rotate motor shaft to given angular position.

    :param position: Position to run servo shaft to (in degrees)
    :param wait_time_ms: Time to wait before continuing the program (in milliseconds). Same effect as ``delay()``, but terminates when servos are disabled.
    :param dps: Speed to run servo at (in degrees per second), from 0 to **max_range**. When dps is 0, servo runs at max speed. Defaults to 0.
    
    .. code-block:: cpp

        //write servo to run to 180 degrees at a speed of 30DPS, and wait 6 seconds
        servo.write(180, 6000, 30);

.. function:: void writeMicroseconds(uint16_t pulse_us, uint16_t wait_time_ms)

    Sends pulse of given length to servo.

    :param pulse_us: Pulse time to transmit to servo (in microseconds) from 200us to 2800us
    :param wait_time_ms: Time to wait before continuing the program (in milliseconds). Same effect as ``delay()``, but terminates when servos are disabled.

    .. code-block:: cpp
        
        //write 1500us pulse to servo, and wait 3 seconds
        servo.writeMicroseconds(1500, 3000);

.. function:: void setMode(bool enable)

    Enable/disable servo. When disabled, the servo will no longer hold its position, and the PIO state machine consumed by the EVNServo object is released.

    :param enable: Whether to enable servo operation
