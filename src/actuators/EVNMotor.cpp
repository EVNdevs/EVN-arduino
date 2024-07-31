#include "EVNMotor.h"

EVNMotor::encoder_state_t* EVNMotor::encoderArgs[];
EVNMotor::pid_control_t* EVNMotor::pidArgs[];
bool EVNMotor::ports_started[] = { false, false, false, false };

EVNMotor::EVNMotor(uint8_t port, uint8_t motortype, uint8_t motor_dir, uint8_t enc_dir)
{
	// clean inputs
	uint8_t motor_dirc = constrain(motor_dir, 0, 1);
	uint8_t enc_dirc = constrain(enc_dir, 0, 1);
	uint8_t portc = constrain(port, 1, 4);
	uint8_t motortypec = constrain(motortype, 0, 3);

	// set pins
	switch (portc)
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
	uint8_t pin;
	if (motor_dirc == REVERSE)
	{
		pin = _pid_control.motora;
		_pid_control.motora = _pid_control.motorb;
		_pid_control.motorb = pin;
		pin = _encoder.enca;
		_encoder.enca = _encoder.encb;
		_encoder.encb = pin;
	}

	if (enc_dirc == REVERSE)
	{
		pin = _encoder.enca;
		_encoder.enca = _encoder.encb;
		_encoder.encb = pin;
	}

	// configure settings according to motor type

	_pid_control.motor_type = motortypec;

	switch (motortypec)
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

	//make sure motor does not run
	_pid_control.run_pwm = false;
	_pid_control.hold = false;
	_pid_control.run_speed = false;
	_pid_control.run_time = false;
	_pid_control.run_pos = false;
	_pid_control.stopAction_static_running = false;
	_pid_control.core0_writing = false;

	_encoder.position = 0;
	_encoder.position_offset = 0;
	_encoder.dps_calculated = false;
	_encoder.obtained_one_pulse = false;
}

void EVNMotor::begin()
{
	//configure pins
	analogWriteFreq(PWM_FREQ);
	analogWriteRange(PWM_MAX_VAL);
	pinMode(_pid_control.motora, OUTPUT_8MA);
	pinMode(_pid_control.motorb, OUTPUT_8MA);
	pinMode(_encoder.enca, INPUT);
	pinMode(_encoder.encb, INPUT);

	//attach pin change interrupts (encoder) and timer interrupt (PID control)
	attach_interrupts(&_encoder, &_pid_control);
}

void EVNMotor::setPID(float p, float i, float d)
{
	_pid_control.pos_pid->setKp(p);
	_pid_control.pos_pid->setKi(i);
	_pid_control.pos_pid->setKd(d);
}

void EVNMotor::setAccel(float accel_dps_sq)
{
	_pid_control.accel = fabs(accel_dps_sq);
}

void EVNMotor::setDecel(float decel_dps_sq)
{
	_pid_control.decel = fabs(decel_dps_sq);
}

void EVNMotor::setMaxRPM(float max_rpm)
{
	_pid_control.max_rpm = fabs(max_rpm);
}

void EVNMotor::setPPR(uint32_t ppr)
{
	_encoder.ppr = ppr;
}

float EVNMotor::getPosition()
{
	return getPosition_static(&_encoder);
}

float EVNMotor::getHeading()
{
	return fmod(fmod(getPosition_static(&_encoder), 360) + 360, 360);
}

void EVNMotor::resetPosition()
{
	_encoder.position_offset = getPosition_static(&_encoder);
}

float EVNMotor::getDPS()
{
	return getDPS_static(&_encoder);
}

float EVNMotor::clean_input_dps(float dps)
{
	return min(fabs(dps), _pid_control.max_rpm * 6);
}

uint8_t EVNMotor::clean_input_dir(float dps)
{
	return (dps >= 0) ? DIRECT : REVERSE;
}

uint8_t EVNMotor::clean_input_stop_action(uint8_t stop_action)
{
	return min(2, stop_action);
}

void EVNMotor::runPWM(float duty_cycle_pct)
{
	while (_pid_control.stopAction_static_running);
	_pid_control.core0_writing = true;

	_pid_control.run_pwm = true;
	_pid_control.run_speed = false;
	_pid_control.run_time = false;
	_pid_control.run_pos = false;
	_pid_control.hold = false;

	_pid_control.core0_writing = false;

	runPWM_static(&_pid_control, constrain(duty_cycle_pct, -100, 100) * 0.01);
}

void EVNMotor::runDPS(float dps)
{
	while (_pid_control.stopAction_static_running);
	_pid_control.core0_writing = true;

	_pid_control.target_dps = clean_input_dps(dps);
	_pid_control.run_dir = clean_input_dir(dps);

	_pid_control.run_pwm = false;
	_pid_control.run_pos = false;
	_pid_control.run_time = false;
	_pid_control.run_speed = true;
	_pid_control.hold = false;

	_pid_control.core0_writing = false;
}

void EVNMotor::runPosition(float dps, float position, uint8_t stop_action, bool wait)
{
	while (_pid_control.stopAction_static_running);
	_pid_control.core0_writing = true;

	_pid_control.target_dps = clean_input_dps(dps);
	_pid_control.target_pos = position;
	_pid_control.stop_action = clean_input_stop_action(stop_action);

	_pid_control.run_pwm = false;
	_pid_control.run_pos = true;
	_pid_control.run_time = false;
	_pid_control.run_speed = false;
	_pid_control.hold = false;

	_pid_control.core0_writing = false;

	if (wait) while (!this->completed());
}

void EVNMotor::runAngle(float dps, float degrees, uint8_t stop_action, bool wait)
{
	float degreesc = degrees;
	if (dps < 0 && degreesc > 0)
		degreesc = -degreesc;

	this->runPosition(dps, this->getPosition() + degreesc, stop_action, wait);
}

void EVNMotor::runHeading(float dps, float heading, uint8_t stop_action, bool wait)
{
	float target_heading = constrain(heading, 0, 360);
	float current_heading = this->getHeading();
	float degreesc = target_heading - current_heading;
	if (degreesc > 180)
		degreesc -= 360;
	if (degreesc < -180)
		degreesc += 360;

	this->runPosition(dps, this->getPosition() + degreesc, stop_action, wait);
}

void EVNMotor::runTime(float dps, uint32_t time_ms, uint8_t stop_action, bool wait)
{
	while (_pid_control.stopAction_static_running);
	_pid_control.core0_writing = true;

	_pid_control.target_dps = clean_input_dps(dps);
	_pid_control.run_dir = clean_input_dir(dps);
	_pid_control.run_time_ms = time_ms;
	_pid_control.stop_action = clean_input_stop_action(stop_action);

	_pid_control.run_pwm = false;
	_pid_control.run_pos = false;
	_pid_control.run_time = true;
	_pid_control.run_speed = false;
	_pid_control.hold = false;

	_pid_control.core0_writing = false;

	if (wait) while (!this->completed());
}

void EVNMotor::stop()
{
	_pid_control.stop_action = STOP_BRAKE;
	stopAction_static(&_pid_control, &_encoder, micros(), getPosition(), getDPS());
}

void EVNMotor::coast()
{
	_pid_control.stop_action = STOP_COAST;
	stopAction_static(&_pid_control, &_encoder, micros(), getPosition(), getDPS());
}

void EVNMotor::hold()
{
	while (_pid_control.stopAction_static_running);
	_pid_control.core0_writing = true;

	_pid_control.target_pos = getPosition_static(&_encoder);
	_pid_control.target_dps = _pid_control.max_rpm * 3;
	_pid_control.stop_action = STOP_HOLD;

	_pid_control.core0_writing = false;

	stopAction_static(&_pid_control, &_encoder, micros(), getPosition(), getDPS());
}

bool EVNMotor::completed()
{
	return (!_pid_control.run_time && !_pid_control.run_pos);
}

bool EVNMotor::stalled()
{
	return _pid_control.stalled && loop_control_enabled(&_pid_control);
}

drivebase_state_t* EVNDrivebase::dbArgs[];
bool EVNDrivebase::dbs_enabled[] = { false, false };

EVNDrivebase::EVNDrivebase(float wheel_dia, float axle_track, EVNMotor* motor_left, EVNMotor* motor_right)
{
	if (motor_left->_pid_control.motor_type == motor_right->_pid_control.motor_type)
		db.motor_type = motor_left->_pid_control.motor_type;
	else
		db.motor_type = CUSTOM_MOTOR;

	db.motor_left = motor_left;
	db.motor_right = motor_right;
	db.max_rpm = min(motor_left->_pid_control.max_rpm, motor_right->_pid_control.max_rpm);

	db.turn_rate_pid = new PIDController(DRIVEBASE_KP_TURN_RATE, DRIVEBASE_KI_TURN_RATE, DRIVEBASE_KD_TURN_RATE, DIRECT);
	db.speed_pid = new PIDController(DRIVEBASE_KP_SPEED, DRIVEBASE_KI_SPEED, DRIVEBASE_KD_SPEED, DIRECT);

	db.speed_accel = fabs(USER_SPEED_ACCEL);
	db.speed_decel = fabs(USER_SPEED_DECEL);
	db.turn_rate_accel = fabs(USER_TURN_RATE_ACCEL);
	db.turn_rate_decel = fabs(USER_TURN_RATE_DECEL);

	db.axle_track = fabs(axle_track);
	db.wheel_dia = fabs(wheel_dia);
	db.drive = false;
	db.drive_position = false;
	db.stopAction_static_running = false;
	db.core0_writing = false;

	db.max_speed = db.max_rpm / 60 * db.wheel_dia * M_PI;
	db.max_turn_rate = db.max_rpm * 6 * db.wheel_dia / axle_track;
	db.max_dps = db.max_rpm * 6;
}

void EVNDrivebase::setSpeedPID(float kp, float ki, float kd)
{
	db.speed_pid->setKp(kp);
	db.speed_pid->setKi(ki);
	db.speed_pid->setKd(kd);
}

void EVNDrivebase::setTurnRatePID(float kp, float ki, float kd)
{
	db.turn_rate_pid->setKp(kp);
	db.turn_rate_pid->setKi(ki);
	db.turn_rate_pid->setKd(kd);
}

void EVNDrivebase::setSpeedAccel(float speed_accel)
{
	db.speed_accel = fabs(speed_accel);
}

void EVNDrivebase::setSpeedDecel(float speed_decel)
{
	db.speed_decel = fabs(speed_decel);
}

void EVNDrivebase::setTurnRateAccel(float turn_rate_accel)
{
	db.turn_rate_accel = fabs(turn_rate_accel);
}

void EVNDrivebase::setTurnRateDecel(float turn_rate_decel)
{
	db.turn_rate_decel = fabs(turn_rate_decel);
}

void EVNDrivebase::begin()
{
	attach_db_interrupt(&db);
}

float EVNDrivebase::getDistance()
{
	return getDistance_static(&db);
}

float EVNDrivebase::getAngle()
{
	return getAngle_static(&db);
}

float EVNDrivebase::getHeading()
{
	return fmod(fmod(getAngle_static(&db), 360) + 360, 360);
}

float EVNDrivebase::getX()
{
	return db.position_x;
}

float EVNDrivebase::getY()
{
	return db.position_y;
}

void EVNDrivebase::resetXY()
{
	db.position_x = 0;
	db.position_y = 0;
}

float EVNDrivebase::getDistanceToPoint(float x, float y)
{
	if (db.position_x == 0 && db.position_y == 0)
		return 0;
	return sqrt(pow(db.position_x - x, 2) + pow(db.position_y - y, 2));
}

float EVNDrivebase::clean_input_turn_rate(float turn_rate)
{
	return constrain(turn_rate, -db.max_turn_rate, db.max_turn_rate);
}

float EVNDrivebase::clean_input_speed(float speed, float turn_rate)
{
	float scale = 1 - fabs(turn_rate) / db.max_turn_rate;
	return constrain(speed, -scale * db.max_speed, scale * db.max_speed);
}

uint8_t EVNDrivebase::clean_input_stop_action(uint8_t stop_action)
{
	return min(2, stop_action);
}

float EVNDrivebase::scaling_factor_for_maintaining_radius(float speed, float turn_rate)
{
	float w1 = fabs(turn_rate / db.max_turn_rate);
	float w2 = fabs(speed / db.max_speed);

	if (w1 + w2 > 1)
		return 1 / (w1 + w2);

	return 1;
}

float EVNDrivebase::radius_to_turn_rate(float speed, float radius)
{
	float radiusc = max(0, radius);
	float turn_rate;
	if (radiusc == 0)
	{
		turn_rate = db.max_turn_rate;
	}
	else
		turn_rate = fabs(speed / M_PI / radiusc * 180);
	return turn_rate;
}


void EVNDrivebase::enable_drive_position(uint8_t stop_action, bool wait)
{
	while (db.stopAction_static_running);
	db.core0_writing = true;

	db.stop_action = clean_input_stop_action(stop_action);
	db.drive = true;
	db.drive_position = true;

	db.core0_writing = false;

	if (wait) while (!this->completed());
}

void EVNDrivebase::drive(float speed, float turn_rate)
{
	this->driveTurnRate(speed, turn_rate);
}

void EVNDrivebase::drivePct(float speed_outer_pct, float turn_rate_pct)
{
	float turn_rate_pctc = constrain(turn_rate_pct, -1, 1);
	float speed_outer_pctc = constrain(speed_outer_pct, -1, 1);
	float speed_inner_pctc = speed_outer_pctc * (1 - 2 * turn_rate_pctc);
	float speed = (speed_outer_pctc + speed_inner_pctc) / 2 * db.max_speed;

	if (turn_rate_pctc < 0)
		speed *= -1;

	if (turn_rate_pctc == 0)
		this->drive(speed, 0);
	else if (turn_rate_pctc == 1)
		this->drive(0, speed_outer_pctc * db.max_turn_rate);
	else if (turn_rate_pctc >= 0.5)
		this->driveRadius(speed, db.axle_track * (1 - fabs(turn_rate_pctc)));
	else
		this->driveRadius(speed, db.axle_track * 0.25 / fabs(turn_rate_pctc));
}

void EVNDrivebase::driveTurnRate(float speed, float turn_rate)
{
	float turn_ratec = clean_input_turn_rate(turn_rate);
	float speedc = clean_input_speed(speed, turn_ratec);

	while (db.stopAction_static_running);
	db.core0_writing = true;

	db.target_speed = speedc;
	db.target_turn_rate = turn_ratec;
	db.drive = true;

	db.core0_writing = false;
}

void EVNDrivebase::driveRadius(float speed, float radius)
{
	float speedc = speed;
	float radiusc = max(0, radius);
	float turn_ratec = radius_to_turn_rate(speed, radius);
	if (radiusc == 0)
		speedc = 0;

	float scale = scaling_factor_for_maintaining_radius(speedc, turn_ratec);
	turn_ratec *= scale;
	speedc *= scale;

	this->driveTurnRate(speedc, turn_ratec);
}

void EVNDrivebase::straight(float speed, float distance, uint8_t stop_action, bool wait)
{
	float speedc = clean_input_speed(speed, 0);
	float distancec = distance;

	if (distancec < 0 && speedc > 0)
		speedc = -speedc;

	if (distancec > 0 && speedc < 0)
		distancec = -distancec;

	db.target_speed = speedc;
	db.target_turn_rate = 0;
	db.end_distance = db.current_distance + distancec;
	db.end_angle = db.current_angle;

	enable_drive_position(stop_action, wait);
}

void EVNDrivebase::curve(float speed, float radius, float angle, uint8_t stop_action, bool wait)
{
	this->curveRadius(speed, radius, angle, stop_action, wait);
}

void EVNDrivebase::curveRadius(float speed, float radius, float angle, uint8_t stop_action, bool wait)
{
	float speedc = speed;
	float radiusc = max(0, radius);
	float turn_ratec = radius_to_turn_rate(speedc, radiusc);
	if (radiusc == 0)
		speedc = 0;

	float scale = scaling_factor_for_maintaining_radius(speedc, turn_ratec);
	turn_ratec *= scale;
	speedc *= scale;

	this->curveTurnRate(speedc, turn_ratec, angle, stop_action, wait);
}

void EVNDrivebase::curveTurnRate(float speed, float turn_rate, float angle, uint8_t stop_action, bool wait)
{
	float turn_ratec = clean_input_turn_rate(turn_rate);
	float speedc = clean_input_speed(speed, turn_ratec);

	if ((angle < 0 && turn_ratec > 0) || (angle > 0 && turn_ratec < 0))
		turn_ratec = -turn_ratec;

	float distance = angle / turn_ratec * speedc;

	db.target_speed = speedc;
	db.target_turn_rate = turn_ratec;
	db.end_distance = db.current_distance + distance;
	db.end_angle = db.current_angle + angle;

	enable_drive_position(stop_action, wait);
}

void EVNDrivebase::turn(float turn_rate, float degrees, uint8_t stop_action, bool wait)
{
	this->turnDegrees(turn_rate, degrees, stop_action, wait);
}

void EVNDrivebase::turnDegrees(float turn_rate, float degrees, uint8_t stop_action, bool wait)
{
	this->curveTurnRate(0, turn_rate, degrees, stop_action, wait);
}

void EVNDrivebase::turnHeading(float turn_rate, float heading, uint8_t stop_action, bool wait)
{
	float target_heading = constrain(heading, 0, 360);
	float current_heading = fmod(fmod(db.current_angle, 360) + 360, 360);
	float turn_angle = heading - current_heading;
	if (turn_angle > 180)
		turn_angle -= 360;
	if (turn_angle < -180)
		turn_angle += 360;

	this->turnDegrees(turn_rate, turn_angle, stop_action, wait);
}

void EVNDrivebase::driveToXY(float speed, float turn_rate, float x, float y, uint8_t stop_action, bool restore_initial_heading)
{
	float angle_to_target = atan2(y - db.position_y, x - db.position_x);
	angle_to_target = angle_to_target / M_PI * 180;
	if (angle_to_target < 0)
		angle_to_target += 360;

	float initial_heading = fmod(fmod(db.current_angle, 360) + 360, 360);
	this->turnHeading(turn_rate, angle_to_target, stop_action, true);
	this->straight(speed, this->getDistanceToPoint(x, y), stop_action, true);
	if (restore_initial_heading)
		this->turnHeading(turn_rate, initial_heading, stop_action, true);
}

void EVNDrivebase::stop()
{
	db.stop_action = STOP_BRAKE;
	stopAction_static(&db);
}

void EVNDrivebase::coast()
{
	db.stop_action = STOP_COAST;
	stopAction_static(&db);
}

void EVNDrivebase::hold()
{
	db.stop_action = STOP_HOLD;
	stopAction_static(&db);
}

bool EVNDrivebase::completed()
{
	return !db.drive_position;
}