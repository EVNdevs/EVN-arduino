#include "EVNMotor.h"

volatile encoder_state_t* EVNMotor::encoderArgs[] = { };
volatile pid_control_t* EVNMotor::pidArgs[] = { };
volatile bool EVNMotor::ports_enabled[] = { };
volatile bool EVNMotor::timerisr_enabled = false;
volatile bool EVNMotor::odom_enabled[] = { };

volatile drivebase_state_t* EVNDrivebase::dbArgs[] = { };
volatile bool EVNDrivebase::dbs_enabled[] = { };
volatile bool EVNDrivebase::odom_enabled[] = { };
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
		_pid_control.unloaded_max_rpm = EV3_LARGE_UNLOADED_MAX_RPM;
		_pid_control.loaded_max_rpm = EV3_LARGE_LOADED_MAX_RPM;
		_pid_control.pwm_mag = EV3_LARGE_PWM_MAG;
		_pid_control.pwm_exp = EV3_LARGE_PWM_EXP;
		_pid_control.accel = EV3_LARGE_ACCEL;
		_pid_control.decel = EV3_LARGE_DECEL;
		_pid_control.pos_pid = new PIDController(EV3_LARGE_KP, 0, EV3_LARGE_KD, DIRECT, 0.5);
		_encoder.ppr = LEGO_PPR;
		break;
	case NXT_LARGE:
		_pid_control.unloaded_max_rpm = NXT_LARGE_UNLOADED_MAX_RPM;
		_pid_control.loaded_max_rpm = NXT_LARGE_LOADED_MAX_RPM;
		_pid_control.pwm_mag = NXT_LARGE_PWM_MAG;
		_pid_control.pwm_exp = NXT_LARGE_PWM_EXP;
		_pid_control.accel = NXT_LARGE_ACCEL;
		_pid_control.decel = NXT_LARGE_DECEL;
		_pid_control.pos_pid = new PIDController(NXT_LARGE_KP, 0, NXT_LARGE_KD, DIRECT, 0.5);
		_encoder.ppr = LEGO_PPR;
		break;
	case EV3_MED:
		_pid_control.unloaded_max_rpm = EV3_MED_UNLOADED_MAX_RPM;
		_pid_control.loaded_max_rpm = EV3_MED_LOADED_MAX_RPM;
		_pid_control.pwm_mag = EV3_MED_PWM_MAG;
		_pid_control.pwm_exp = EV3_MED_PWM_EXP;
		_pid_control.accel = EV3_MED_ACCEL;
		_pid_control.decel = EV3_MED_DECEL;
		_pid_control.pos_pid = new PIDController(EV3_MED_KP, 0, EV3_MED_KD, DIRECT, 0.5);
		_encoder.ppr = LEGO_PPR;
		break;
	case CUSTOM_MOTOR:
		_pid_control.unloaded_max_rpm = CUSTOM_UNLOADED_MAX_RPM;
		_pid_control.loaded_max_rpm = CUSTOM_LOADED_MAX_RPM;
		_pid_control.pwm_mag = CUSTOM_PWM_MAG;
		_pid_control.pwm_exp = CUSTOM_PWM_EXP;
		_pid_control.accel = CUSTOM_ACCEL;
		_pid_control.decel = CUSTOM_DECEL;
		_pid_control.pos_pid = new PIDController(CUSTOM_KP, 0, CUSTOM_KD, DIRECT, 0.5);
		_encoder.ppr = CUSTOM_PPR;
		break;
	}

	compute_ppr_derived_values_unsafe();
	 _pid_control.hold_done = true;
}

void EVNMotor::begin() volatile
{
	while (!EVNAlpha::started())
	{
		delay(50);
	}
	EVNCoreSync0.begin();
	compute_max_rpm_unsafe();

	//configure pins
	analogWriteFreq(PWM_FREQ);
	analogWriteRange(PWM_MAX_VAL);
	pinMode(_pid_control.motora, OUTPUT);
	pinMode(_pid_control.motorb, OUTPUT);
	pinMode(_encoder.enca, INPUT_PULLUP);
	pinMode(_encoder.encb, INPUT_PULLUP);

	//attach pin change interrupts (encoder) and timer interrupt (PID control)
	attach_interrupts(&_encoder, &_pid_control);
}

void EVNMotor::setKp(float kp) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.pos_pid->setKp(kp);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::setKd(float kd) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.pos_pid->setKd(kd);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::setPWMMapping(float mag, float exp) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.pwm_mag = max(0, mag);
	_pid_control.pwm_exp = max(0, exp);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::setAccel(float accel_dps_per_s) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.accel = fabs(accel_dps_per_s);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::setDecel(float decel_dps_per_s) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.decel = fabs(decel_dps_per_s);
	_pid_control.max_error_before_decel = _pid_control.target_dps * _pid_control.target_dps / _pid_control.decel / 2;
	_pid_control.decel_dps_per_us = _pid_control.decel / 1000000;
	_pid_control.time_to_decel_us = _pid_control.target_dps / _pid_control.decel * 1000000;

	EVNCoreSync0.core0_exit();
}

void EVNMotor::setMaxRPM(float unloaded_max_rpm, float loaded_max_rpm) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.unloaded_max_rpm = fabs(unloaded_max_rpm);
	if (loaded_max_rpm <= 0)
		_pid_control.loaded_max_rpm = _pid_control.unloaded_max_rpm;
	else
		_pid_control.loaded_max_rpm = min(loaded_max_rpm, _pid_control.unloaded_max_rpm);
	compute_max_rpm_unsafe();

	EVNCoreSync0.core0_exit();
}

void EVNMotor::setPPR(float ppr) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_encoder.ppr = ppr;
	compute_ppr_derived_values_unsafe();

	EVNCoreSync0.core0_exit();
}

void EVNMotor::setDebug(bool enable) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_pid_control.debug = enable;

	EVNCoreSync0.core0_exit();
}

float EVNMotor::getKp() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = _pid_control.pos_pid->getKp();

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::getKd() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = _pid_control.pos_pid->getKd();

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::getPWMMag() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = _pid_control.pwm_mag;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::getPWMExp() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = _pid_control.pwm_exp;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::getAccel() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = _pid_control.accel;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::getDecel() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = _pid_control.decel;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::getMaxRPM() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = _pid_control.max_rpm;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::getMaxDPS() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = _pid_control.max_rpm;

	EVNCoreSync0.core0_exit();

	return output * 6;
}

float EVNMotor::getPPR() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = _encoder.ppr;

	EVNCoreSync0.core0_exit();

	return output;
}

bool EVNMotor::getDebug() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = _pid_control.debug;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::getError() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = _pid_control.error;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::getPosition() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = getPosition_static(&_encoder);

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNMotor::get_target_position() volatile
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

float EVNMotor::get_target_heading() volatile
{
	return fmod(fmod(get_target_position(), 360) + 360, 360);
}

void EVNMotor::disable_connected_drivebase_unsafe() volatile
{
	for (int i = 0; i < EVNDrivebase::MAX_DB_OBJECTS; i++)
		if (EVNDrivebase::odom_enabled[i])
			if (EVNDrivebase::dbArgs[i]->motor_left == this || EVNDrivebase::dbArgs[i]->motor_right == this)
				EVNDrivebase::set_drivebase_mode_unsafe(i, false);
}

void EVNMotor::setPosition(float position) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	_encoder.position_offset = ((float)_encoder.count * 90.0 / _encoder.ppr) - position;

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

float EVNMotor::clean_input_dps_unsafe(float dps) volatile
{
	return min(fabs(dps), _pid_control.max_rpm * 6);
}

uint8_t EVNMotor::clean_input_dir(float dps) volatile
{
	return (dps >= 0) ? DIRECT : REVERSE;
}

uint8_t EVNMotor::clean_input_stop_action(uint8_t stop_action) volatile
{
	return min(MAX_STOP_VALUE, stop_action);
}

void EVNMotor::compute_ppr_derived_values_unsafe() volatile
{
	_encoder._90_div_ppr = 90 / _encoder.ppr;
	_encoder._360000000_div_ppr = 360000000 / _encoder.ppr;
}

void EVNMotor::compute_max_rpm_unsafe() volatile
{
	//set max rpm based on battery voltage and loaded + unloaded speeds at full charge (8.4V)
	float vbatt_on_boot = (float) EVNAlpha::getBatteryVoltageOnBoot_unsafe();
	if (vbatt_on_boot == 0)
		vbatt_on_boot = 8400;
	float voltage_drop_from_load = 8400 * (1 - _pid_control.loaded_max_rpm / _pid_control.unloaded_max_rpm);
	float scaling_factor = (vbatt_on_boot - voltage_drop_from_load) / (8400 - voltage_drop_from_load);
	_pid_control.max_rpm = _pid_control.loaded_max_rpm * scaling_factor;
	_pid_control.max_rpm_calculated = false;
}

void EVNMotor::runPWM_unsafe(float duty_cycle) volatile
{
	_pid_control.run_pwm = true;
	_pid_control.run_speed = false;
	_pid_control.run_time = false;
	_pid_control.run_pos = false;

	runPWM_static(&_pid_control, duty_cycle);
}

void EVNMotor::runPWM(float duty_cycle_pct) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();
	disable_connected_drivebase_unsafe();

	float duty_cycle = constrain(duty_cycle_pct, -100, 100) / 100;
	runPWM_unsafe(duty_cycle);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::runSpeed(float dps, bool disable_accel_decel) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	runSpeed_unsafe(dps, disable_accel_decel);

	EVNCoreSync0.core0_exit();
}

void EVNMotor::runSpeed_unsafe(float dps, bool disable_accel_decel) volatile
{
	disable_connected_drivebase_unsafe();

	_pid_control.target_dps = clean_input_dps_unsafe(dps);
	_pid_control.run_dir = clean_input_dir(dps);

	_pid_control.run_pwm = false;
	_pid_control.run_pos = false;
	_pid_control.run_time = false;
	_pid_control.run_speed = true;
	_pid_control.accel_decel_off = disable_accel_decel;
}

void EVNMotor::runPosition(float dps, float position, uint8_t stop_action, bool wait) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();
	disable_connected_drivebase_unsafe();

	_pid_control.target_dps = clean_input_dps_unsafe(dps);
	_pid_control.end_pos = position;
	_pid_control.stop_action = clean_input_stop_action(stop_action);

	_pid_control.run_pwm = false;
	_pid_control.run_pos = true;
	_pid_control.run_time = false;
	_pid_control.run_speed = false;
	_pid_control.accel_decel_off = false;

	_pid_control.max_error_before_decel = _pid_control.target_dps * _pid_control.target_dps / _pid_control.decel / 2;

	EVNCoreSync0.core0_exit();

	if (wait)
		while (!this->completed());
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

	this->runPosition(dps, this->get_target_position() + degrees, stop_action, wait);
}

void EVNMotor::runHeading(float dps, float heading, uint8_t stop_action, bool wait) volatile
{
	heading = constrain(heading, 0, 360);
	float degrees = heading - this->get_target_heading();
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
	disable_connected_drivebase_unsafe();

	_pid_control.target_dps = clean_input_dps_unsafe(dps);
	_pid_control.run_dir = clean_input_dir(dps);
	_pid_control.start_time_us = micros();
	_pid_control.run_time_us = time_ms * 1000;
	_pid_control.stop_action = clean_input_stop_action(stop_action);

	_pid_control.run_pwm = false;
	_pid_control.run_pos = false;
	_pid_control.run_time = true;
	_pid_control.run_speed = false;
	_pid_control.accel_decel_off = false;

	_pid_control.decel_dps_per_us = _pid_control.decel / 1000000;
	_pid_control.time_to_decel_us = _pid_control.target_dps / _pid_control.decel * 1000000;

	EVNCoreSync0.core0_exit();

	if (wait)
		while (!this->completed());
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
	_pid_control.stop_action = STOP_HOLD;
	stopAction_static(&_pid_control, &_encoder, getPosition_static(&_encoder), getDPS_static(&_encoder));
}

void EVNMotor::stop() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	disable_connected_drivebase_unsafe();
	stop_unsafe();

	EVNCoreSync0.core0_exit();
}

void EVNMotor::coast() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	disable_connected_drivebase_unsafe();
	coast_unsafe();

	EVNCoreSync0.core0_exit();
}

void EVNMotor::hold() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	disable_connected_drivebase_unsafe();
	hold_unsafe();

	EVNCoreSync0.core0_exit();

	while (!this->completed());
}

bool EVNMotor::completed() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	bool output = !_pid_control.run_time && !_pid_control.run_pos && _pid_control.hold_done;

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
	{
		db.motor_type = db.motor_left->_pid_control.motor_type;
		float kp = db.motor_left->_pid_control.pos_pid->getKp();
		float kd = db.motor_left->_pid_control.pos_pid->getKd();
		db.turn_rate_pid = new PIDController(kp, 0, kd, DIRECT, 0.5);
		db.speed_pid = new PIDController(kp, 0, kd, DIRECT, 0.5);
	}
	else
	{
		db.motor_type = CUSTOM_MOTOR;
		db.turn_rate_pid = new PIDController(CUSTOM_KP, 0, CUSTOM_KD, DIRECT, 0.5);
		db.speed_pid = new PIDController(CUSTOM_KP, 0, CUSTOM_KD, DIRECT, 0.5);
	}

	db.speed_accel = fabs(DRIVEBASE_SPEED_ACCEL);
	db.speed_decel = fabs(DRIVEBASE_SPEED_DECEL);
	db.turn_rate_accel = fabs(DRIVEBASE_TURN_RATE_ACCEL);
	db.turn_rate_decel = fabs(DRIVEBASE_TURN_RATE_DECEL);

	compute_drivebase_derived_values_unsafe(&db);
	compute_max_rpm_drivebase_derived_values_unsafe(&db);
	compute_targets_decel_derived_values_unsafe(&db);

	db.hold_done = true;
}

void EVNDrivebase::begin() volatile
{
	EVNCoreSync0.begin();
	attach_interrupts(&db);
}

void EVNDrivebase::setSpeedKp(float kp) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.speed_pid->setKp(kp);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setSpeedKd(float kd) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.speed_pid->setKd(kd);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setTurnRateKp(float kp) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.turn_rate_pid->setKp(kp);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setTurnRateKd(float kd) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.turn_rate_pid->setKd(kd);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setSpeedAccel(float accel_mm_per_s_sq) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.speed_accel = fabs(accel_mm_per_s_sq);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setSpeedDecel(float decel_mm_per_s_sq) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.speed_decel = fabs(decel_mm_per_s_sq);
	compute_targets_decel_derived_values_unsafe(&db);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setTurnRateAccel(float accel_deg_per_s_sq) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.turn_rate_accel = fabs(accel_deg_per_s_sq);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setTurnRateDecel(float decel_deg_per_s_sq) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.turn_rate_decel = fabs(decel_deg_per_s_sq);
	compute_targets_decel_derived_values_unsafe(&db);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setDebug(uint8_t debug_type) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.debug = constrain(debug_type, 0, 2);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setAxleTrack(float axle_track) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.axle_track = fabs(axle_track);
	compute_drivebase_derived_values_unsafe(&db);
	compute_max_rpm_drivebase_derived_values_unsafe(&db);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::setWheelDia(float wheel_dia) volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	db.wheel_dia = fabs(wheel_dia);
	compute_drivebase_derived_values_unsafe(&db);
	compute_max_rpm_drivebase_derived_values_unsafe(&db);

	EVNCoreSync0.core0_exit();
}

float EVNDrivebase::getSpeedKp() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.speed_pid->getKp();

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getSpeedKd() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.speed_pid->getKd();

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getTurnRateKp() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.turn_rate_pid->getKp();

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getTurnRateKd() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.turn_rate_pid->getKd();

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getSpeedAccel() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.speed_accel;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getSpeedDecel() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.speed_decel;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getTurnRateAccel() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.turn_rate_accel;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getTurnRateDecel() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.turn_rate_decel;

	EVNCoreSync0.core0_exit();

	return output;
}

uint8_t EVNDrivebase::getDebug() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	uint8_t output = db.debug;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getAxleTrack() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.axle_track;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getWheelDia() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.wheel_dia;

	EVNCoreSync0.core0_exit();

	return output;
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

	float output = getAngleRad_static(&db) * RAD_TO_DEG;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getHeading() volatile
{
	return fmod(fmod(getAngle(), 360) + 360, 360);
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

float EVNDrivebase::getMaxSpeed() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.max_speed;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::getMaxTurnRate() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.max_speed;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::get_target_angle() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	float output = db.target_angle;

	EVNCoreSync0.core0_exit();

	return output;
}

float EVNDrivebase::get_target_heading() volatile
{
	return fmod(fmod(get_target_angle(), 360) + 360, 360);
}

float EVNDrivebase::clean_input_turn_rate_unsafe(float turn_rate) volatile
{
	return constrain(turn_rate, -db.max_turn_rate, db.max_turn_rate);
}

float EVNDrivebase::clean_input_speed_unsafe(float speed, float turn_rate) volatile
{
	float scale = 1 - fabs(turn_rate) / db.max_turn_rate;
	return constrain(speed, -scale * db.max_speed, scale * db.max_speed);
}

uint8_t EVNDrivebase::clean_input_stop_action(uint8_t stop_action) volatile
{
	return min(MAX_STOP_VALUE, stop_action);
}

float EVNDrivebase::scaling_factor_for_maintaining_radius_unsafe(float speed, float turn_rate) volatile
{
	float w1 = fabs(turn_rate / db.max_turn_rate);
	float w2 = fabs(speed / db.max_speed);

	if (w1 + w2 > 1)
		return 1 / (w1 + w2);

	return 1;
}

float EVNDrivebase::radius_to_turn_rate_unsafe(float speed, float radius) volatile
{
	if (radius == 0)
		return db.max_turn_rate;

	return fabs(speed) * 180 / (M_PI * radius);
}

void EVNDrivebase::drive(float speed, float turn_rate, bool disable_accel_decel) volatile
{
	this->driveTurnRate(speed, turn_rate, disable_accel_decel);
}

void EVNDrivebase::drivePct(float speed_outer_pct, float turn_rate_pct, bool disable_accel_decel) volatile
{
	if (!timerisr_enabled) return;

	turn_rate_pct = constrain(turn_rate_pct, -100, 100) / 100;
	speed_outer_pct = constrain(speed_outer_pct, -100, 100) / 100;
	float speed_inner_pct = speed_outer_pct * (1 - 2 * fabs(turn_rate_pct));

	EVNCoreSync0.core0_enter();

	float speed = (speed_outer_pct + speed_inner_pct) / 2 * db.max_speed;
	float unsigned_turn_rate_rad = db.max_speed * (speed_outer_pct - speed_inner_pct) / db.axle_track;

	EVNCoreSync0.core0_exit();

	float turn_rate_rad = unsigned_turn_rate_rad  * (turn_rate_pct >= 0 ? 1 : -1);
	float turn_rate = turn_rate_rad * 180 / M_PI;
	this->drive(speed, turn_rate, disable_accel_decel);
}

void EVNDrivebase::driveTurnRate(float speed, float turn_rate, bool disable_accel_decel) volatile
{
	turn_rate = clean_input_turn_rate_unsafe(turn_rate);
	speed = clean_input_speed_unsafe(speed, turn_rate);

	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();
	set_drivebase_mode_unsafe(db.id, true);

	db.target_speed = speed;
	db.target_turn_rate = turn_rate;
	db.drive = true;
	db.drive_position = false;
	db.accel_decel_off = disable_accel_decel;

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::driveRadius(float speed, float radius, bool disable_accel_decel) volatile
{
	float turn_rate = radius_to_turn_rate_unsafe(speed, radius);
	if (radius == 0)
		speed = 0;

	float scale = scaling_factor_for_maintaining_radius_unsafe(speed, turn_rate);
	turn_rate *= scale;
	speed *= scale;

	this->driveTurnRate(speed, turn_rate, disable_accel_decel);
}

void EVNDrivebase::straight(float speed, float distance, uint8_t stop_action, bool wait) volatile
{	
	speed = clean_input_speed_unsafe(speed, 0);

	if (distance == 0 || speed == 0)
		return;

	if (distance < 0 || speed < 0)
	{
		distance = -fabs(distance);
		speed = -fabs(speed);
	}

	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();
	set_drivebase_mode_unsafe(db.id, true);

	db.end_angle = db.target_angle;
	db.end_distance = db.target_distance + distance;
	db.target_speed = speed;
	db.target_turn_rate = 0;
	db.stop_action = clean_input_stop_action(stop_action);
	db.drive = true;
	db.drive_position = true;
	db.accel_decel_off = false;

	compute_targets_decel_derived_values_unsafe(&db);

	EVNCoreSync0.core0_exit();

	if (wait)
		while (!this->completed());
}

void EVNDrivebase::curve(float speed, float radius, float angle, uint8_t stop_action, bool wait) volatile
{
	this->curveRadius(speed, radius, angle, stop_action, wait);
}

void EVNDrivebase::curveRadius(float speed, float radius, float angle, uint8_t stop_action, bool wait) volatile
{
	float turn_rate = radius_to_turn_rate_unsafe(speed, radius);

	if (radius == 0)
		speed = 0;

	float scale = scaling_factor_for_maintaining_radius_unsafe(speed, turn_rate);
	turn_rate *= scale;
	speed *= scale;

	this->curveTurnRate(speed, turn_rate, angle, stop_action, wait);
}

void EVNDrivebase::curveTurnRate(float speed, float turn_rate, float angle, uint8_t stop_action, bool wait) volatile
{
	turn_rate = clean_input_turn_rate_unsafe(turn_rate);
	speed = clean_input_speed_unsafe(speed, turn_rate);

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
	set_drivebase_mode_unsafe(db.id, true);

	db.end_angle = db.target_angle + angle;
	db.end_distance = db.target_distance + distance;
	db.target_speed = speed;
	db.target_turn_rate = turn_rate;
	db.stop_action = clean_input_stop_action(stop_action);
	db.drive = true;
	db.drive_position = true;
	db.accel_decel_off = false;

	compute_targets_decel_derived_values_unsafe(&db);

	EVNCoreSync0.core0_exit();

	if (wait)
		while (!this->completed());
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
	float turn_angle = heading - get_target_heading();
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

	float initial_heading = get_target_heading();
	this->turnHeading(turn_rate, angle_to_target, stop_action, true);
	this->straight(speed, this->getDistanceToPoint(x, y), stop_action, true);
	if (restore_initial_heading)
		this->turnHeading(turn_rate, initial_heading, stop_action, true);
}

void EVNDrivebase::stop() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	set_drivebase_mode_unsafe(db.id, true);
	db.stop_action = STOP_BRAKE;
	stopAction_static(&db);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::coast() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	set_drivebase_mode_unsafe(db.id, true);
	db.stop_action = STOP_COAST;
	stopAction_static(&db);

	EVNCoreSync0.core0_exit();
}

void EVNDrivebase::hold() volatile
{
	if (!timerisr_enabled) return;
	EVNCoreSync0.core0_enter();

	set_drivebase_mode_unsafe(db.id, true);
	db.stop_action = STOP_HOLD;
	stopAction_static(&db);

	EVNCoreSync0.core0_exit();

	while (!this->completed());
}

bool EVNDrivebase::completed() volatile
{
	if (!timerisr_enabled) return 0;
	EVNCoreSync0.core0_enter();

	bool output = !db.drive_position && db.hold_done;

	EVNCoreSync0.core0_exit();

	return output;
}