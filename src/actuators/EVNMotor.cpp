#include "EVNMotor.h"

volatile encoder_state_t* EVNMotor::encoderArgs[] = { };
volatile pid_control_t* EVNMotor::pidArgs[] = { };
volatile bool EVNMotor::ports_started[] = { };
volatile bool EVNMotor::timerisr_enabled = false;

volatile drivebase_state_t* EVNDrivebase::dbArgs[] = { };
volatile bool EVNDrivebase::dbs_started[] = { };
volatile bool EVNDrivebase::timerisr_enabled = false;

EVNMotor::EVNMotor(uint8_t port, uint8_t motor_type, uint8_t motor_dir, uint8_t enc_dir)
{
	// clean inputs
	motor_dir = constrain(motor_dir, 0, 1);
	enc_dir = constrain(enc_dir, 0, 1);
	_pid_control.port = constrain(port, 1, 4);
	motor_type = constrain(motor_type, 0, 3);

	// set pins
	switch (_pid_control.port)
	{
	case 1:
		_pid_control.motora = PIN_MOTOR1_OUTA;
		_pid_control.motorb = PIN_MOTOR1_OUTB;
		_encoder.enca = PIN_MOTOR1_ENCA;
		_encoder.encb = PIN_MOTOR1_ENCB;
		break;
	case 2:
		_pid_control.motora = PIN_MOTOR2_OUTA;
		_pid_control.motorb = PIN_MOTOR2_OUTB;
		_encoder.enca = PIN_MOTOR2_ENCA;
		_encoder.encb = PIN_MOTOR2_ENCB;
		break;
	case 3:
		_pid_control.motora = PIN_MOTOR3_OUTA;
		_pid_control.motorb = PIN_MOTOR3_OUTB;
		_encoder.enca = PIN_MOTOR3_ENCA;
		_encoder.encb = PIN_MOTOR3_ENCB;
		break;
	case 4:
		_pid_control.motora = PIN_MOTOR4_OUTA;
		_pid_control.motorb = PIN_MOTOR4_OUTB;
		_encoder.enca = PIN_MOTOR4_ENCA;
		_encoder.encb = PIN_MOTOR4_ENCB;
		break;
	}

	// swap pins if needed
	uint8_t pin = 0;

	if (motor_dir == REVERSE)
	{
		pin = _pid_control.motora;
		_pid_control.motora = _pid_control.motorb;
		_pid_control.motorb = pin;
		pin = _encoder.enca;
		_encoder.enca = _encoder.encb;
		_encoder.encb = pin;
	}

	if (enc_dir == REVERSE)
	{
		pin = _encoder.enca;
		_encoder.enca = _encoder.encb;
		_encoder.encb = pin;
	}

	// configure settings according to motor type
	_pid_control.motor_type = motor_type;

	switch (motor_type)
	{
	case EV3_LARGE:
		_pid_control.max_rpm = EV3_LARGE_MAX_RPM;
		_pid_control.accel = EV3_LARGE_ACCEL;
		_pid_control.decel = EV3_LARGE_DECEL;
		_pid_control.pos_pid = new PIDController(KP_EV3_LARGE, KI_EV3_LARGE, KD_MAX_EV3_LARGE, DIRECT);
		_encoder.ppr = LEGO_PPR;
		break;
	case NXT_LARGE:
		_pid_control.max_rpm = NXT_LARGE_MAX_RPM;
		_pid_control.accel = NXT_LARGE_ACCEL;
		_pid_control.decel = NXT_LARGE_DECEL;
		_pid_control.pos_pid = new PIDController(KP_NXT_LARGE, KI_NXT_LARGE, KD_MAX_NXT_LARGE, DIRECT);
		_encoder.ppr = LEGO_PPR;
		break;
	case EV3_MED:
		_pid_control.max_rpm = EV3_MED_MAX_RPM;
		_pid_control.accel = EV3_MED_ACCEL;
		_pid_control.decel = EV3_MED_DECEL;
		_pid_control.pos_pid = new PIDController(KP_EV3_MED, KI_EV3_MED, KD_MAX_EV3_MED, DIRECT);
		_encoder.ppr = LEGO_PPR;
		break;
	case CUSTOM_MOTOR:
		_pid_control.max_rpm = CUSTOM_MOTOR_MAX_RPM;
		_pid_control.accel = CUSTOM_MOTOR_ACCEL;
		_pid_control.decel = CUSTOM_MOTOR_DECEL;
		_pid_control.pos_pid = new PIDController(KP_CUSTOM, KI_CUSTOM, KD_MAX_CUSTOM, DIRECT);
		_encoder.ppr = CUSTOM_PPR;
		break;
	}
}

void EVNMotor::begin() volatile
{
	EVNCoreSync0.begin();

	if (timerisr_enabled)
		EVNCoreSync0.core0_enter();
	else
		EVNCoreSync0.core0_enter_force();

	//configure pins
	analogWriteFreq(PWM_FREQ);
	analogWriteRange(PWM_MAX_VAL);
	pinMode(_pid_control.motora, OUTPUT);
	pinMode(_pid_control.motorb, OUTPUT);
	pinMode(_encoder.enca, INPUT_PULLUP);
	pinMode(_encoder.encb, INPUT_PULLUP);

	//attach pin change interrupts (encoder) and timer interrupt (PID control)
	attach_interrupts(&_encoder, &_pid_control);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::end() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync1.core0_enter();

	ports_started[_pid_control.port - 1] = false;

	EVNCoreSync1.core0_exit();
}

void EVNMotor::setPID(float p, float i, float d) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.pos_pid->setKp(p);
	_pid_control.pos_pid->setKi(i);
	_pid_control.pos_pid->setKd(d);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::setAccel(float accel_dps_sq) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.accel = fabs(accel_dps_sq);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::setDecel(float decel_dps_sq) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.decel = fabs(decel_dps_sq);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::setMaxRPM(float max_rpm) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.max_rpm = fabs(max_rpm);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::setPPR(uint32_t ppr) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_encoder.ppr = ppr;

	EVNCoreSync0.core0_exit();
}

void EVNMotor::setDebug(bool enable) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.debug = enable;

	EVNCoreSync0.core0_exit();
}

float EVNMotor::getPosition() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = getPosition_static(&_encoder);

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::getTargetPosition() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = _pid_control.target_pos;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::getHeading() volatile
{
	return fmod(fmod(getPosition(), 360) + 360, 360);
}

float EVNMotor::getTargetHeading() volatile
{
	return fmod(fmod(getTargetPosition(), 360) + 360, 360);
}

void EVNMotor::setPosition(float position) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_encoder.position_offset = ((float)_encoder.position * 90.0 / _encoder.ppr) - position;

	EVNCoreSync0.core0_exit();
}

void EVNMotor::resetPosition() volatile
{
	this->setPosition(0);
}

float EVNMotor::getSpeed() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = getDPS_static(&_encoder);

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::clean_input_dps(float dps) volatile
{
	return min(fabs(dps), _pid_control.max_rpm * 6);
}

uint8_t EVNMotor::clean_input_dir(float dps) volatile
{
	return (dps >= 0) ? DIRECT : REVERSE;
}

uint8_t EVNMotor::clean_input_stop_action(uint8_t stop_action) volatile
{
	return min(2, stop_action);
}

void EVNMotor::runPWM(float duty_cycle_pct) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	duty_cycle_pct = constrain(duty_cycle_pct, -100, 100) / 100;

	_pid_control.run_pwm = true;
	_pid_control.run_speed = false;
	_pid_control.run_time = false;
	_pid_control.run_pos = false;

	runPWM_static(&_pid_control, duty_cycle_pct);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::runSpeed(float dps) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	runSpeed_unsafe(dps);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::runSpeed_unsafe(float dps) volatile
{
	_pid_control.target_dps = clean_input_dps(dps);
	_pid_control.run_dir = clean_input_dir(dps);

	_pid_control.run_pwm = false;
	_pid_control.run_pos = false;
	_pid_control.run_time = false;
	_pid_control.run_speed = true;
}

void EVNMotor::runPosition(float dps, float position, uint8_t stop_action, bool wait) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.target_dps = clean_input_dps(dps);
	_pid_control.end_pos = position;
	_pid_control.stop_action = clean_input_stop_action(stop_action);

	_pid_control.run_pwm = false;
	_pid_control.run_pos = true;
	_pid_control.run_time = false;
	_pid_control.run_speed = false;

	EVNCoreSync0.core0_exit();

	if (wait) while (!this->completed());
}

void EVNMotor::runAngle(float dps, float degrees, uint8_t stop_action, bool wait) volatile
{
	if (dps == 0 || degrees == 0)
		return;

	if (dps < 0 || degrees < 0)
	{
		dps = -fabs(dps);
		degrees = -fabs(degrees);
	}

	this->runPosition(dps, this->getTargetPosition() + degrees, stop_action, wait);
}

void EVNMotor::runHeading(float dps, float heading, uint8_t stop_action, bool wait) volatile
{
	heading = constrain(heading, 0, 360);
	float degrees = heading - this->getTargetHeading();
	if (degrees > 180)
		degrees -= 360;
	if (degrees < -180)
		degrees += 360;

	this->runAngle(dps, degrees, stop_action, wait);
}

void EVNMotor::runTime(float dps, uint32_t time_ms, uint8_t stop_action, bool wait) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.target_dps = clean_input_dps(dps);
	_pid_control.run_dir = clean_input_dir(dps);
	_pid_control.start_time_us = micros();
	_pid_control.run_time_ms = time_ms;
	_pid_control.stop_action = clean_input_stop_action(stop_action);

	_pid_control.run_pwm = false;
	_pid_control.run_pos = false;
	_pid_control.run_time = true;
	_pid_control.run_speed = false;

	EVNCoreSync0.core0_exit();

	if (wait) while (!this->completed());
}

void EVNMotor::stop_unsafe() volatile
{
	_pid_control.stop_action = STOP_BRAKE;
	stopAction_static(&_pid_control, &_encoder, getPosition_static(&_encoder), getDPS_static(&_encoder));
}

void EVNMotor::coast_unsafe() volatile
{
	_pid_control.stop_action = STOP_BRAKE;
	stopAction_static(&_pid_control, &_encoder, getPosition_static(&_encoder), getDPS_static(&_encoder));
}

void EVNMotor::hold_unsafe() volatile
{
	_pid_control.end_pos = getPosition_static(&_encoder);
	_pid_control.stop_action = STOP_HOLD;
	stopAction_static(&_pid_control, &_encoder, getPosition_static(&_encoder), getDPS_static(&_encoder));
}

void EVNMotor::stop() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	stop_unsafe();

	EVNCoreSync0.core0_exit();
}

void EVNMotor::coast() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	coast_unsafe();

	EVNCoreSync0.core0_exit();
}

void EVNMotor::hold() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	hold_unsafe();

	EVNCoreSync0.core0_exit();
}

bool EVNMotor::completed() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	bool output = !_pid_control.run_time && !_pid_control.run_pos;

	EVNCoreSync0.core0_exit();

	return output;
}

bool EVNMotor::stalled_unsafe() volatile
{
	return _pid_control.stalled && loop_control_enabled(&_pid_control);
}

bool EVNMotor::stalled() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	bool output = _pid_control.stalled && loop_control_enabled(&_pid_control);

	EVNCoreSync0.core0_exit();

	return output;
}

EVNDrivebase::EVNDrivebase(float wheel_dia, float axle_track, EVNMotor* motor_left, EVNMotor* motor_right)
{
	db.motor_left = motor_left;
	db.motor_right = motor_right;

	db.axle_track = fabs(axle_track);
	db.wheel_dia = fabs(wheel_dia);


	if (db.motor_left->_pid_control.motor_type == db.motor_right->_pid_control.motor_type)
		db.motor_type = db.motor_left->_pid_control.motor_type;
	else
		db.motor_type = CUSTOM_MOTOR;

	db.max_rpm = min(db.motor_left->_pid_control.max_rpm, db.motor_right->_pid_control.max_rpm);
	db.max_speed = db.max_rpm / 60 * db.wheel_dia * M_PI;
	db.max_turn_rate = db.max_rpm * 6 * db.wheel_dia / db.axle_track;
	db.max_dps = db.max_rpm * 6;
	db.max_distance_error = USER_DRIVE_POS_MIN_ERROR_MOTOR_DEG * db.wheel_dia * M_PI / 360;
	db.max_angle_error = USER_DRIVE_POS_MIN_ERROR_MOTOR_DEG * db.wheel_dia / db.axle_track;

	db.turn_rate_pid = new PIDController(DRIVEBASE_KP_TURN_RATE, DRIVEBASE_KI_TURN_RATE, DRIVEBASE_KD_TURN_RATE, DIRECT);
	db.speed_pid = new PIDController(DRIVEBASE_KP_SPEED, DRIVEBASE_KI_SPEED, DRIVEBASE_KD_SPEED, DIRECT);

	db.speed_accel = fabs(USER_SPEED_ACCEL);
	db.speed_decel = fabs(USER_SPEED_DECEL);
	db.turn_rate_accel = fabs(USER_TURN_RATE_ACCEL);
	db.turn_rate_decel = fabs(USER_TURN_RATE_DECEL);
}

void EVNDrivebase::begin() volatile
{
	EVNCoreSync0.begin();
	if (timerisr_enabled || EVNMotor::timerisr_enabled)
		EVNCoreSync0.core0_enter();
	else
		EVNCoreSync0.core0_enter_force();

	attach_interrupts(&db);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::end() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync1.core0_enter();

	dbs_started[db.id - 1] = false;

	EVNCoreSync1.core0_exit();
}

void EVNDrivebase::setSpeedPID(float kp, float ki, float kd) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.speed_pid->setKp(kp);
	db.speed_pid->setKi(ki);
	db.speed_pid->setKd(kd);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setTurnRatePID(float kp, float ki, float kd) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.turn_rate_pid->setKp(kp);
	db.turn_rate_pid->setKi(ki);
	db.turn_rate_pid->setKd(kd);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setSpeedAccel(float speed_accel) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.speed_accel = fabs(speed_accel);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setSpeedDecel(float speed_decel) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.speed_decel = fabs(speed_decel);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setTurnRateAccel(float turn_rate_accel) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.turn_rate_accel = fabs(turn_rate_accel);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setTurnRateDecel(float turn_rate_decel) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.turn_rate_decel = fabs(turn_rate_decel);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setDebug(bool enable) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.debug = enable;

	EVNCoreSync0.core0_exit();
}

float EVNDrivebase::getDistance() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = getDistance_static(&db);

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getAngle() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = getAngle_static(&db);

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getTargetAngle() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.target_angle;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getHeading() volatile
{
	return fmod(fmod(getAngle(), 360) + 360, 360);
}

float EVNDrivebase::getTargetHeading() volatile
{
	return fmod(fmod(getTargetAngle(), 360) + 360, 360);
}

float EVNDrivebase::getX() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.position_x;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getY() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.position_y;

	EVNCoreSync0.core0_exit();

	return output;
}

void EVNDrivebase::resetXY() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.position_x = 0;
	db.position_y = 0;

	EVNCoreSync0.core0_exit();
}

float EVNDrivebase::getDistanceToPoint(float x, float y) volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float px = db.position_x;
	float py = db.position_y;

	EVNCoreSync0.core0_exit();

	if (py == 0 && px == 0)
		return 0;

	return sqrt(pow(px - x, 2) + pow(py - y, 2));
}

float EVNDrivebase::clean_input_turn_rate(float turn_rate) volatile
{
	return constrain(turn_rate, -db.max_turn_rate, db.max_turn_rate);
}

float EVNDrivebase::clean_input_speed(float speed, float turn_rate) volatile
{
	float scale = 1 - fabs(turn_rate) / db.max_turn_rate;
	return constrain(speed, -scale * db.max_speed, scale * db.max_speed);
}

uint8_t EVNDrivebase::clean_input_stop_action(uint8_t stop_action) volatile
{
	return min(2, stop_action);
}

float EVNDrivebase::scaling_factor_for_maintaining_radius(float speed, float turn_rate) volatile
{
	float w1 = fabs(turn_rate / db.max_turn_rate);
	float w2 = fabs(speed / db.max_speed);

	if (w1 + w2 > 1)
		return 1 / (w1 + w2);

	return 1;
}

float EVNDrivebase::radius_to_turn_rate(float speed, float radius) volatile
{
	if (radius == 0)
		return db.max_turn_rate;

	return fabs(speed) * 180 / (M_PI * radius);
}

void EVNDrivebase::stall_until_stopped() volatile
{
	if (!timerisr_enabled) return;

	bool stop = false;

	while (!stop)
	{
		EVNCoreSync0.core0_enter();
		stop = db.stall_until_stop;
		EVNCoreSync0.core0_exit();
	}
}

void EVNDrivebase::drive(float speed, float turn_rate) volatile
{
	this->driveTurnRate(speed, turn_rate);
}

void EVNDrivebase::drivePct(float speed_outer_pct, float turn_rate_pct) volatile
{
	turn_rate_pct = constrain(turn_rate_pct, -100, 100) / 100;
	speed_outer_pct = constrain(speed_outer_pct, -100, 100) / 100;
	float speed_inner_pct = speed_outer_pct * (1 - 2 * fabs(turn_rate_pct));
	float speed = (speed_outer_pct + speed_inner_pct) / 2 * db.max_speed;

	//drive straight
	if (turn_rate_pct == 0)
		this->drive(speed, 0);

	//turn on the spot
	else if (fabs(turn_rate_pct) == 1)
		this->drive(0, fabs(speed_outer_pct) * db.max_turn_rate * turn_rate_pct);

	//centre of turning radius between wheels
	else if (turn_rate_pct >= 0.5)
		this->driveRadius(speed, db.axle_track * (1 - turn_rate_pct));

	else if (turn_rate_pct <= -0.5)
		this->driveRadius(speed, -db.axle_track * (1 + turn_rate_pct));

	//centre of turning radius outside of wheels
	else
		this->driveRadius(speed, db.axle_track * 0.25 / turn_rate_pct);
}

void EVNDrivebase::driveTurnRate(float speed, float turn_rate) volatile
{
	turn_rate = clean_input_turn_rate(turn_rate);
	speed = clean_input_speed(speed, turn_rate);

	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.target_speed = speed;
	db.target_turn_rate = turn_rate;
	db.drive = true;
	db.drive_position = false;

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::driveRadius(float speed, float radius) volatile
{
	float turn_rate = radius_to_turn_rate(speed, radius);
	if (radius == 0)
		speed = 0;

	float scale = scaling_factor_for_maintaining_radius(speed, turn_rate);
	turn_rate *= scale;
	speed *= scale;

	this->driveTurnRate(speed, turn_rate);
}

void EVNDrivebase::straight(float speed, float distance, uint8_t stop_action, bool wait) volatile
{
	speed = clean_input_speed(speed, 0);

	if (distance == 0 || speed == 0)
		return;

	if (distance < 0 || speed < 0)
	{
		distance = -fabs(distance);
		speed = -fabs(speed);
	}

	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.end_angle = db.target_angle;
	db.end_distance = db.target_distance + distance;
	db.target_speed = speed;
	db.target_turn_rate = 0;
	db.stop_action = clean_input_stop_action(stop_action);
	db.drive = true;
	db.drive_position = true;

	EVNCoreSync0.core0_exit();

	if (wait)
	{
		while (!this->completed());
		stall_until_stopped();
	}
}

void EVNDrivebase::curve(float speed, float radius, float angle, uint8_t stop_action, bool wait) volatile
{
	this->curveRadius(speed, radius, angle, stop_action, wait);
}

void EVNDrivebase::curveRadius(float speed, float radius, float angle, uint8_t stop_action, bool wait) volatile
{
	float turn_rate = radius_to_turn_rate(speed, radius);

	if (radius == 0)
		speed = 0;

	float scale = scaling_factor_for_maintaining_radius(speed, turn_rate);
	turn_rate *= scale;
	speed *= scale;

	this->curveTurnRate(speed, turn_rate, angle, stop_action, wait);
}

void EVNDrivebase::curveTurnRate(float speed, float turn_rate, float angle, uint8_t stop_action, bool wait) volatile
{
	turn_rate = clean_input_turn_rate(turn_rate);
	speed = clean_input_speed(speed, turn_rate);

	if (turn_rate == 0 || angle == 0)
		return;

	if (angle < 0 || turn_rate < 0)
	{
		angle = -fabs(angle);
		turn_rate = -fabs(turn_rate);
	}

	float distance = angle / turn_rate * speed;

	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.end_angle = db.target_angle + angle;
	db.end_distance = db.target_distance + distance;
	db.target_speed = speed;
	db.target_turn_rate = turn_rate;
	db.stop_action = clean_input_stop_action(stop_action);
	db.drive = true;
	db.drive_position = true;

	EVNCoreSync0.core0_exit();

	if (wait)
	{
		while (!this->completed());
		stall_until_stopped();
	}
}

void EVNDrivebase::turn(float turn_rate, float degrees, uint8_t stop_action, bool wait) volatile
{
	this->turnDegrees(turn_rate, degrees, stop_action, wait);
}

void EVNDrivebase::turnDegrees(float turn_rate, float degrees, uint8_t stop_action, bool wait) volatile
{
	this->curveTurnRate(0, turn_rate, degrees, stop_action, wait);
}

void EVNDrivebase::turnHeading(float turn_rate, float heading, uint8_t stop_action, bool wait) volatile
{
	heading = constrain(heading, 0, 360);
	float turn_angle = heading - getTargetHeading();
	if (turn_angle > 180)
		turn_angle -= 360;
	if (turn_angle < -180)
		turn_angle += 360;

	this->turnDegrees(turn_rate, turn_angle, stop_action, wait);
}

void EVNDrivebase::driveToXY(float speed, float turn_rate, float x, float y, uint8_t stop_action, bool restore_initial_heading) volatile
{
	float angle_to_target = atan2(y - db.position_y, x - db.position_x);
	angle_to_target = angle_to_target / M_PI * 180;
	if (angle_to_target < 0)
		angle_to_target += 360;

	float initial_heading = getTargetHeading();
	this->turnHeading(turn_rate, angle_to_target, stop_action, true);
	this->straight(speed, this->getDistanceToPoint(x, y), stop_action, true);
	if (restore_initial_heading)
		this->turnHeading(turn_rate, initial_heading, stop_action, true);
}

void EVNDrivebase::stop() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.stop_action = STOP_BRAKE;
	stopAction_static(&db);

	EVNCoreSync0.core0_exit();

	stall_until_stopped();
}

void EVNDrivebase::coast() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.stop_action = STOP_COAST;
	stopAction_static(&db);

	EVNCoreSync0.core0_exit();

	stall_until_stopped();
}

void EVNDrivebase::hold() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.stop_action = STOP_HOLD;
	stopAction_static(&db);

	EVNCoreSync0.core0_exit();

	stall_until_stopped();
}

bool EVNDrivebase::completed() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	bool output = !db.drive_position;

	EVNCoreSync0.core0_exit();

	return output;
}