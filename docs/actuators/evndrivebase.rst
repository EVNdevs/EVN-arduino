``EVNDrivebase``
================

Building upon the functionality provided by EVNMotor, EVNDrivebase provides functions useful for controlling differential drive robots.

.. note:: EVNDrivebase consumes some of the RP2040's spinlock peripherals. See this `page`_ for more info.

.. _page: ../getting-started/hardware-overview.html

.. note::
    EVNDrivebase uses the 2nd core to perform internal updates at a high rate. Between updates, user code can run on the 2nd core,
    but this configuration may cause timing-sensitive code to fail or lead to unexpected behaviour.

Constructor
-----------

.. class:: EVNDrivebase(float wheel_dia, float axle_track, EVNMotor* motor_left, EVNMotor* motor_right);

    :param wheel_dia: Diameter of each wheel (in mm)
    :param axle_track: Distance between midpoint of both wheels (in mm)
    :param motor_left: Pointer to ``EVNMotor`` object for left motor
    :param motor_right: Pointer to ``EVNMotor`` object for right motor

    .. code-block:: cpp

        EVNMotor left_motor(1, EV3_MED);
        EVNMotor right_motor(2, EV3_MED, REVERSE);

        //wheel diameter - 62.4mm, axle track - 178mm
        EVNDrivebase db(62.4, 178, &left_motor, &right_motor);

.. note::

    For those new to C++, pointers can be quite confusing! Think of ``&`` as a means for you to provide a link to an object. 
    Here, we pass a link to ``EVNMotor`` objects to our ``EVNDrivebase`` object, so that it can control the ``EVNMotor`` objects for us.

Functions
---------

.. function:: void begin();

    Initializes drivebase object. 
    Call this function after calling ``begin()`` on the EVNMotor objects (which still need to be called!), 
    but before calling any other ``EVNDrivebase`` functions (including settings functions).

    .. code-block:: cpp

        EVNMotor left_motor(1, EV3_MED);
        EVNMotor right_motor(2, EV3_MED, REVERSE);
        EVNDrivebase db(62.4, 178, &left_motor, &right_motor);

        void setup1()
        {
            left_motor.begin();
            right_motor.begin();
            db.begin();
        }

.. note::
    EVNDrivebase **should** be initialized on the 2nd core by calling ``begin()`` in ``void setup1()``. 
    However, you can still call the other functions on both cores.
    We recommend setting drivebase parameters on the 2nd core, while leaving movement to the main core.

Measurements
""""""""""""

.. function:: float getDistance()

    :returns: Distance travelled by drivebase (in mm)

    .. code-block:: cpp

        float distance = db.getDistance();

.. function:: float getAngle()

    :returns: Angle turned by drivebase (in deg)
    
    .. code-block:: cpp

        float angle = db.getAngle();

.. function:: float getHeading()

    :returns: Drivebase heading (ranging from 0-360 degrees)

    .. code-block:: cpp

        float heading = db.getHeading();

.. function:: float getX()

    :returns: X coordinate of drivebase from origin (origin is the drivebase's position on startup)

    .. code-block:: cpp

        float x = db.getX();

.. function:: float getY()

    :returns: Y coordinate of drivebase from origin (origin is the drivebase's position on startup)

    .. code-block:: cpp

        float y = db.getY();

.. function:: void resetXY();

    Sets drivebase's current position to be the origin (0, 0).

    .. code-block:: cpp
        
        db.resetXY();
        //afterwards, getX() and getY() will return 0

.. function:: float getDistanceToPoint(float x, float y);

    :returns: Euclidean distance between drivebase's XY position and target XY point

    .. code-block:: cpp

        //if drivebase is at origin, the distance to point will be 4
        float distance_to_point = db.getDistanceToPoint(3,2);

.. function:: float getMaxSpeed()

    :returns: Maximum speed of drivebase (in mm/s)

.. function:: float getMaxTurnRate()

    :returns: Maximum turning rate of drivebase (in deg/s)

Move Forever
""""""""""""

.. function::   void drive(float speed, float turn_rate, bool disable_accel_decel = false);
                void driveTurnRate(float speed, float turn_rate, bool disable_accel_decel = false);

    Runs drivebase at the given speed and turn rate until a new command is called

    :param speed: Velocity of drivebase (in mm/s)
    :param turn_rate: Turning rate of drivebase (in deg/s)
    :param disable_accel_decel: Disables acceleration and deceleration control. Defaults to `false`

    .. code-block:: cpp
        
        //drive at a velocity of 50mm/s and turning rate of 5deg/s
        db.drive(50, 5);

.. function:: void driveRadius(float speed, float radius, bool disable_accel_decel = false);

    Runs drivebase at the given speed and radius of turning until a new command is called

    :param speed: Velocity of drivebase (in mm/s)
    :param radius: Turning radius of drivebase (in mm)
    :param disable_accel_decel: Disables acceleration and deceleration control. Defaults to `false`

    .. code-block:: cpp
        
        //drive at a velocity of 50mm/s and move in an arc of radius 50mm
        db.driveRadius(50, 50);

.. function:: void drivePct(float speed_outer_pct, float turn_rate_pct, bool disable_accel_decel = true)
    
    This function simulates a differential drive function where the outer wheel speed is given as a percentage, 
    along with a "turning rate" percentage input to define the turning behaviour as described below:

    0% "turning rate": Both wheels run at speed_outer_pct, same direction
    50% "turning rate": One wheel stationary, other wheel runs at speed_outer_pct (one-wheel turn)
    100% "turning rate": Both wheels run at speed_outer_pct, but in opposite directions

    Positive turning rate values turn anti-clockwise, negative values turn clockwise.

    Since the inputs range from -100 to 100 (unlike driveRadius, where radius ranges to infinity), 
    it can be easier to use this function for PID control.

    :param speed_outer_pct: Speed for outer (faster) wheel in % (number from -100 to 100)
    :param turn_rate_pct: Turning rate of drivebase in % (number from -100 to 100)
    :param disable_accel_decel: Disables acceleration and deceleration control. Defaults to `true`

    .. code-block:: cpp
        
        //drive forwards at 100% speed
        db.drivePct(100, 0);
        //drive forwards at 50% speed
        db.drivePct(50, 0);
        //drive backwards at 100% speed
        db.drivePct(-100, 0);
        //one-wheel clockwise turn at full speed
        db.drivePct(100, 50);
        //rotate clockwise on the spot at full speed
        db.drivePct(100, 100);

Move by a Fixed Amount
""""""""""""""""""""""

.. function:: void straight(float speed, float distance, uint8_t stop_action = STOP_BRAKE, bool wait = true);

    Runs drivebase in a straight line for the specified distance, then performs given stop action.

    Drivebase speed direction is reversed when the ``speed`` or ``distance`` inputs are negative 
    (e.g. ``straight(100, -100)``, ``straight(-100, 100)``, or
    ``straight(-100, -100)`` will all move the drivebase backwards).

    :param speed: Velocity of drivebase (in mm/s)
    :param distance: Distance to travel (in mm)
    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position
        * ``STOP_NONE`` -- Continue running at current speed (and deceleration to a stop removed)

    :param wait: Block function from returning until command is finished

    .. code-block:: cpp
        
        //move in a straight line at a velocity of 50mm/s for a distance of 50mm
        db.straight(50, 50);

.. function::   void curve(float speed, float radius, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true);
                void curveRadius(float speed, float radius, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true);

    Runs drivebase in a curve of specified radius until its heading has shifted by the given angle, then performs given stop action.

    Drivebase turning direction is reversed when the ``radius`` or ``angle`` inputs are negative 
    (e.g. ``curve(100, 100, -100)``, ``curve(100, -100, 100)``, or
    ``curve(100, -100, -100)`` will all use a negative (clockwise) turning rate).

    :param speed: Velocity of drivebase (in mm/s)
    :param radius: Turning radius of drivebase (in mm)
    :param angle: Angle to travel by (in deg)
    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position
        * ``STOP_NONE`` -- Continue running at current speed (and deceleration to a stop removed)

    :param wait: Block function from returning until command is finished

    .. code-block:: cpp
        
        //drive at a velocity of 50mm/s in an arc of radius 50mm until the drivebase has rotated by 90 degrees
        db.curve(50, 50, 90, STOP_BRAKE);

.. function:: void curveTurnRate(float speed, float turn_rate, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true);

    Runs drivebase at given speed and turn rate until its heading has shifted by the given angle, then runs specified stop action

    Drivebase turning direction is reversed when the ``turn_rate`` or ``angle`` inputs are negative 
    (e.g. ``curveTurnRate(100, 100, -100)``, ``curveTurnRate(100, -100, 100)``,
    or ``curveTurnRate(100, -100, -100)`` will all use a negative (clockwise) turning rate).

    :param speed: Velocity of drivebase (in mm/s)
    :param turn_rate: Turning rate of drivebase (in deg/s)
    :param angle: Angle to travel by (in deg)
    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position
        * ``STOP_NONE`` -- Continue running at current speed (and deceleration to a stop removed)

    :param wait: Block function from returning until command is finished

    .. code-block:: cpp
        
        //drive at a velocity of 50mm/s at a turning rate of 5deg/s until the drivebase has rotated by 90 degrees
        db.curveTurnRate(50, 5, 90, STOP_BRAKE);

.. function::   void turn(float turn_rate, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true);
                void turnDegrees(float turn_rate, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true);

    Rotate drivebase on the spot by the given angle, then performs given stop action
    
    :param turn_rate: Turning rate of drivebase (in deg/s)
    :param angle: Angle to travel by (in deg)
    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position
        * ``STOP_NONE`` -- Continue running at current speed (and deceleration to a stop removed)

    :param wait: Block function from returning until command is finished

    .. code-block:: cpp
        
        //rotate at a rate of 5deg/s until the drivebase has rotated by 90 degrees
        db.turn(5, 90, STOP_BRAKE);

.. function:: void turnHeading(float turn_rate, float heading, uint8_t stop_action = STOP_BRAKE, bool wait = true);

    Rotate drivebase on the spot to the given heading, then performs given stop action

    :param turn_rate: Turning rate of drivebase (in deg/s)
    :param heading: Heading to travel to (in deg)
    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position
        * ``STOP_NONE`` -- Continue running at current speed (and deceleration to a stop removed)

    :param wait: Block function from returning until command is finished

    .. code-block:: cpp
        
        //rotate at a rate of 5deg/s (or -5deg/s) until the drivebase has a heading of 90degrees
        db.turnHeading(5, 90, STOP_BRAKE);    

.. function:: bool completed();

    :returns: Boolean indicating whether the drivebase's command has reached completion

    .. code-block:: cpp

        //wait until drivebase has completed its command
        while (!db.completed());

Move to Point
""""""""""""""""
.. function:: void driveToXY(float speed, float turn_rate, float x, float y, uint8_t stop_action = STOP_BRAKE, bool restore_initial_heading = true);

    Rotates drivebase to face target XY position, drives forward to target, and rotates back to original heading

    :param speed: Velocity of drivebase (in mm/s)
    :param turn_rate: Turning rate of drivebase (in deg/s)
    :param x: X coordinate of target
    :param y: Y coordinate of target
    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position
        * ``STOP_NONE`` -- Continue running at current speed (and deceleration to a stop removed)

    :param wait: Block function from returning until command is finished

    .. code-block:: cpp
        
        //drive to point (60, 60) at a velocity of 100mm/s and turning rate of 50deg/s
        db.driveToXY(100, 50, 60, 60, STOP_BRAKE);

.. note:: This feature is experimental! And also, it's pretty much a party trick. Its behaviour may be changed in future versions.

Stopping
""""""""

.. function::   void stop()

    Brakes both drivebase motors (slow decay). 

    .. code-block:: cpp
        
        db.stop();

.. function:: void coast()
    
    Coasts both drivebase motors to a stop (fast decay). 
    Compared to braking with `stop()`, the drivebase comes to a stop more slowly.
    
    .. code-block:: cpp
    
        db.coast();

.. function:: void hold()
    
    Hold drivebase motors in their current positions. Restricts the motor shafts from moving.

    .. code-block:: cpp
    
        db.hold();

Control Settings
""""""""""""""""

To view the default accel/decel values, look at ``src\evn_motor_defs.h`` in the Github repository.

The PD values for the speed and turn rate controllers are obtained from the motor objects linked to drivebase.

If both motors are the same motor type, then the EVNMotor PD value for that motor type will be used by default.
Otherwise, the CUSTOM_MOTOR PD values are used.

.. function:: void setSpeedKp(float kp);

    Sets proportional gain values for the speed controller (controls average drivebase speed).

    The error for the controller is the difference between the robot's target distance travelled
    and the robot's current distance travelled (in motor degrees).

    :param kp: Proportional gain

    .. code-block:: cpp
    
        db.setSpeedKp(20);

.. function:: void setSpeedKd(float kd);

    Sets derivative gain values for the speed controller (controls average drivebase speed).

    The error for the controller is the difference between the robot's target distance travelled
    and the robot's current distance travelled (in motor degrees).

    :param kd: Derivative gain

    .. code-block:: cpp
    
        db.setSpeedKd(0.2);

.. function:: void setTurnRateKp(float kp);

    Sets proportional gain values for the turn rate controller (controls rate of turning of drivebase).

    The error for the controller is the difference between the robot's target angle
    and the robot's current angle (in motor degrees).

    This controller serves 2 purposes: to ensure the drivebase turns at the correct rate,
    and to stop either motor if the other is stalled, syncing their movement.

    :param kp: Proportional gain

    .. code-block:: cpp
    
        db.setTurnRateKp(20);

.. function:: void setTurnRateKd(float kd);

    Sets derivative gain values for the turn rate controller (controls rate of turning of drivebase).

    The error for the controller is the difference between the robot's target angle
    and the robot's current angle (in motor degrees).

    This controller serves 2 purposes: to ensure the drivebase turns at the correct rate,
    and to stop either motor if the other is stalled, syncing their movement.

    :param kd: Derivative gain

    .. code-block:: cpp
    
        db.setTurnRateKd(0.2);

.. function:: void setSpeedAccel(float accel_mm_per_s_sq);

    Sets speed acceleration value for drivebase (in mm/s^2).

    :param accel_mm_per_s_sq: Acceleration in mm/s^2

    .. code-block:: cpp
    
        db.setSpeedAccel(500);

.. function:: void setSpeedDecel(float decel_mm_per_s_sq);

    Sets speed deceleration value for drivebase (in mm/s^2).

    :param decel_mm_per_s_sq: Deceleration in mm/s^2

    .. code-block:: cpp
    
        db.setSpeedDecel(500);

.. function:: void setTurnRateAccel(float accel_deg_per_s_sq);

    Sets turn rate acceleration value for drivebase (in deg/s^2).

    :param accel_deg_per_s_sq: Acceleration in deg/s^2

    .. code-block:: cpp
    
        db.setTurnRateAccel(500);

.. function:: void setTurnRateDecel(float decel_deg_per_s_sq);

    Sets turn rate deceleration value for drivebase (in deg/s^2).

    :param decel_deg_per_s_sq: Deceleration in deg/s^2

    .. code-block:: cpp
    
        db.setTurnRateDecel(500);

.. function:: void setDebug(uint8_t debug_type)

    Used to toggle debug mode, where drivebase will print the error for either speed or
    turn rate PD control over ``Serial``. Can be used to observe or tune PD controller behaviour.

    :param debug_type: Type of debug mode to run

        * ``DEBUG_OFF``
        * ``DEBUG_SPEED``
        * ``DEBUG_TURN_RATE``