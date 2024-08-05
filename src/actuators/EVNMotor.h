#ifndef EVNMotor_h
#define EVNMotor_h

#include <Arduino.h>
#include "../EVNAlpha.h"
#include "../helper/EVNISRTimer.h"
#include "../helper/PIDController.h"
#include "../evn_motor_defs.h"

//INPUT PARAMETER MACROS
#define DIRECT	1
#define REVERSE	0

#define EV3_LARGE		0
#define NXT_LARGE		1
#define EV3_MED			2
#define CUSTOM_MOTOR	3

#define STOP_BRAKE		0
#define STOP_COAST		1
#define STOP_HOLD		2

//DPS MEASUREMENT (TIME BETWEEN PULSES)
#define NO_OF_EDGES_STORED 3

class EVNMotor
{
	struct encoder_state_t
	{
		//POSITION MEASUREMENT
		volatile uint8_t enca;
		volatile uint8_t encb;
		volatile bool enca_state;
		volatile bool encb_state;
		volatile int8_t dir;
		volatile uint8_t state;
		volatile float ppr;
		volatile float position;
		volatile float position_offset;

		//DPS MEASUREMENT (TIME BETWEEN PULSES)
		volatile uint8_t last_edge_index;
		volatile uint64_t edge_times[NO_OF_EDGES_STORED];
		volatile bool dps_calculated;
		volatile bool obtained_one_pulse;
		volatile float avg_dps;
		volatile float avg_pulse_width;
	};

	struct pid_control_t
	{
		//MOTOR CHARACTERISTICS
		volatile uint8_t motor_type;
		uint8_t motora;
		uint8_t motorb;
		volatile float max_rpm;
		volatile float accel;
		volatile float decel;

		//CONTROLLER
		PIDController* pos_pid;

		//USER-SET VARIABLES
		volatile bool stopAction_static_running;
		volatile bool core0_writing;
		volatile bool run_pwm;
		volatile bool run_speed;
		volatile bool run_dir;
		volatile float target_dps;
		volatile bool run_pos;
		volatile float target_pos;
		volatile bool run_time;
		volatile uint32_t run_time_ms;
		volatile bool hold;
		volatile uint8_t stop_action;

		//LOOP VARIABLES
		volatile uint64_t last_update;
		volatile float target_dps_end_decel;
		volatile float target_dps_constrained;
		volatile float x;
		volatile float error;
		volatile float output;

		volatile uint8_t counter;
		volatile uint64_t start_time_us;

		volatile bool stalled;
	};

public:
	static const uint32_t PWM_FREQ = 20000;
	static const uint32_t PWM_MAX_VAL = 255;
	static const uint16_t PID_TIMER_INTERVAL_US = 1000;
	static const uint32_t ENCODER_PULSE_TIMEOUT_US = 166667 * 2;

	friend class EVNDrivebase;
	friend class EVNOmniDrivebaseBasic;

	EVNMotor(uint8_t port, uint8_t motortype = EV3_LARGE, uint8_t motor_dir = DIRECT, uint8_t enc_dir = DIRECT);
	void begin();
	float getPosition();
	float getHeading();
	void resetPosition();
	float getSpeed();

	void runPWM(float duty_cycle_pct);
	void runSpeed(float dps);
	void runPosition(float dps, float position, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void runAngle(float dps, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void runHeading(float dps, float heading, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void runTime(float dps, uint32_t time_ms, uint8_t stop_action = STOP_BRAKE, bool wait = true);

	void stop();
	void coast();
	void hold();

	bool completed();
	bool stalled();

	void setPID(float p, float i, float d);
	void setAccel(float accel_dps_sq);
	void setDecel(float decel_dps_sq);
	void setMaxRPM(float max_rpm);
	void setPPR(uint32_t ppr);

	//TODO: add end function

protected:
	float clean_input_dps(float dps);
	uint8_t clean_input_dir(float dps);
	uint8_t clean_input_stop_action(uint8_t stop_action);

	float _position_offset = 0;
	pid_control_t _pid_control;
	encoder_state_t _encoder;

	static bool ports_started[4];
	static encoder_state_t* encoderArgs[4]; // static list of pointers to each instances' structs
	static pid_control_t* pidArgs[4];

	static bool motors_enabled()
	{
		return ((EVNAlpha::sharedButtonLED().read() && EVNAlpha::sharedButtonLED().sharedState()->link_movement)

			|| !EVNAlpha::sharedButtonLED().sharedState()->link_movement);
	}

	static bool timed_control_enabled(pid_control_t* arg)
	{
		return arg->run_time;
	}

	static bool position_control_enabled(pid_control_t* arg)
	{
		return arg->run_pos || arg->hold;
	}

	static bool loop_control_enabled(pid_control_t* arg)
	{
		return (arg->run_speed || arg->run_time || arg->run_pos || arg->hold);
	}

	static void velocity_update(encoder_state_t* arg, uint64_t now)
	{
		if (arg->enca_state)
		{
			arg->last_edge_index++;
			arg->last_edge_index %= NO_OF_EDGES_STORED;
			arg->edge_times[arg->last_edge_index] = now;

			if (!arg->obtained_one_pulse && arg->last_edge_index == 2)
				arg->obtained_one_pulse = true;

			arg->dps_calculated = false;
		}
	}

	static void pos_update(encoder_state_t* arg)
	{
		uint8_t state = arg->state & 3;

		arg->enca_state = digitalRead(arg->enca);
		arg->encb_state = digitalRead(arg->encb);

		if (arg->enca_state)
			state |= 4;
		if (arg->encb_state)
			state |= 8;
		arg->state = (state >> 2);

		switch (state)
		{
		case 1: case 7: case 8: case 14:
			arg->position++;
			arg->dir = 1;
			return;
		case 2: case 4: case 11: case 13:
			arg->position--;
			arg->dir = -1;
			return;
		case 3: case 12:
			arg->position += 2;
			arg->dir = 1;
			return;
		case 6: case 9:
			arg->position -= 2;
			arg->dir = -1;
			return;
		}
	}

	static void runPWM_static(pid_control_t* pidArg, float speed)
	{

		float speedc = constrain(speed, -1, 1);

		// if button is set to enable motors / button is unlinked from motors
		if (motors_enabled()) {
			if (speedc == 0)
			{
				digitalWrite(pidArg->motora, LOW);
				digitalWrite(pidArg->motorb, LOW);
			}
			else if (speedc == 1)
			{
				digitalWrite(pidArg->motora, HIGH);
				digitalWrite(pidArg->motorb, LOW);
			}
			else if (speedc == -1)
			{
				digitalWrite(pidArg->motora, LOW);
				digitalWrite(pidArg->motorb, HIGH);
			}
			else if (speedc > 0)
			{
				digitalWrite(pidArg->motorb, LOW);
				analogWrite(pidArg->motora, speedc * PWM_MAX_VAL);
			}
			else
			{
				digitalWrite(pidArg->motora, LOW);
				analogWrite(pidArg->motorb, -speedc * PWM_MAX_VAL);
			}
		}
	}

	static void stopAction_static(pid_control_t* pidArg, encoder_state_t* encoderArg, uint64_t now, float pos, float dps)
	{
		if (pidArg->core0_writing) return;
		pidArg->stopAction_static_running = true;

		//keep start_time and x at most recent states
		pidArg->start_time_us = now;
		pidArg->target_dps_constrained = dps;

		//coast and brake stop functions are based on DRV8833 datasheet
		//hold uses position PID control, setting the current position as an endpoint
		switch (pidArg->stop_action)
		{
		case STOP_COAST:
			digitalWrite(pidArg->motora, LOW);
			digitalWrite(pidArg->motorb, LOW);
			pidArg->x = pos;
			pidArg->hold = false;
			break;
		case STOP_BRAKE:
			digitalWrite(pidArg->motora, HIGH);
			digitalWrite(pidArg->motorb, HIGH);
			pidArg->x = pos;
			pidArg->hold = false;
			break;
		case STOP_HOLD:
			pidArg->hold = true;
			break;
		}

		//reset PID controller, stop loop control
		pidArg->pos_pid->reset();
		pidArg->run_pwm = false;
		pidArg->run_speed = false;
		pidArg->run_pos = false;
		pidArg->run_time = false;

		pidArg->stopAction_static_running = false;
	}

	static float getPosition_static(encoder_state_t* arg)
	{
		// resolution of encoder readout is in CPR (4 * PPR)
		return (float)arg->position * 360 / (4 * arg->ppr) - arg->position_offset;
	}

	static float getDPS_static(encoder_state_t* arg)
	{
		uint64_t now = micros();
		float last_pulse_width = now - arg->edge_times[arg->last_edge_index];

		if (!arg->obtained_one_pulse) return 0;

		if (!arg->dps_calculated)
		{
			float sum_dps = 0;
			float sum_pulse_width = 0;
			float number_of_pulses = 0;

			for (uint8_t i = 0; i < (NO_OF_EDGES_STORED - 1); i++)
			{
				uint8_t end_index = (arg->last_edge_index - i + NO_OF_EDGES_STORED) % NO_OF_EDGES_STORED;
				uint8_t start_index = (end_index - 1 + NO_OF_EDGES_STORED) % NO_OF_EDGES_STORED;
				uint64_t end = arg->edge_times[end_index];
				uint64_t start = arg->edge_times[start_index];

				float pulse_width = end - start;
				float pulse_dps = 0;
				if (pulse_width > 0)
				{
					pulse_dps = 360000000 / pulse_width / arg->ppr;
					number_of_pulses++;
				}

				sum_pulse_width += pulse_width;
				sum_dps += pulse_dps;
			}

			arg->avg_pulse_width = sum_pulse_width / number_of_pulses;
			arg->avg_dps = sum_dps / number_of_pulses;
			arg->dps_calculated = true;
		}

		// if timeout, DPS is 0
		if (last_pulse_width > ENCODER_PULSE_TIMEOUT_US) return 0;

		// if latest pulse is longer than prior pulses, use latest pulse
		// this occurs when the motor is slowing down (pulses get longer and longer)
		else if (last_pulse_width > arg->avg_pulse_width)
		{
			float final_dps = (1000000 / last_pulse_width) / arg->ppr * 360;
			return final_dps * arg->dir;
		}

		else
			return arg->avg_dps * arg->dir;
	}

	static void pid_update(pid_control_t* pidArg, encoder_state_t* encoderArg)
	{
		uint64_t now = micros();
		float pos = getPosition_static(encoderArg);
		float dps = getDPS_static(encoderArg);
		float time_since_last_loop_scaled = ((float)now - (float)pidArg->last_update) / 1000;
		float time_since_last_loop = time_since_last_loop_scaled / 1000;
		pidArg->last_update = now;

		if (motors_enabled() && loop_control_enabled(pidArg))
		{
			float decel_dps = pidArg->target_dps;

			if (position_control_enabled(pidArg))
			{
				if (!pidArg->hold)
				{
					if (fabs(pidArg->target_pos - pos) <= USER_RUN_DEGREES_MIN_ERROR_DEG)
						pidArg->counter++;
					else
						pidArg->counter = 0;

					if (pidArg->counter >= USER_RUN_DEGREES_MIN_LOOP_COUNT)
					{
						stopAction_static(pidArg, encoderArg, now, pos, dps);
						return;
					}
				}

				pidArg->run_dir = (pidArg->target_pos - pos > 0) ? DIRECT : REVERSE;

				float error = fabs(pidArg->target_pos - pos);
				float max_error_before_decel = pow(fabs(pidArg->target_dps), 2) / pidArg->decel / 2;

				if (error < max_error_before_decel)
					decel_dps = sqrt(error / max_error_before_decel) * fabs(pidArg->target_dps);
			}

			if (pidArg->run_time)
			{
				if ((now - pidArg->start_time_us) >= pidArg->run_time_ms * 1000)
				{
					stopAction_static(pidArg, encoderArg, now, pos, dps);
					return;
				}

				if ((pidArg->target_dps / pidArg->decel) * 1000000 > ((float)pidArg->run_time_ms * 1000 + pidArg->start_time_us - now))
					decel_dps = pidArg->decel * ((float)pidArg->run_time_ms * 1000 + (float)pidArg->start_time_us - (float)now) / 1000000;
			}

			pidArg->target_dps_end_decel = decel_dps;
			float signed_target_dps_end_decel = pidArg->target_dps_end_decel * ((pidArg->run_dir == DIRECT) ? 1 : -1);

			float kp = pidArg->pos_pid->getKp();
			float ki = pidArg->pos_pid->getKi();
			float kd = pidArg->pos_pid->getKd();

			if (!position_control_enabled(pidArg) || pidArg->stalled)
				pidArg->pos_pid->setKi(0);
			else
				pidArg->pos_pid->setKi(ki * time_since_last_loop_scaled);

			if (fabs((pidArg->x - pos) * pidArg->pos_pid->getKp() + pidArg->pos_pid->getIntegral() * pidArg->pos_pid->getKi()) < 1) //anti-windup
			{
				pidArg->stalled = false;

				if (pidArg->target_dps_constrained <= signed_target_dps_end_decel)
				{
					if (pidArg->target_dps_constrained > 0)
						pidArg->target_dps_constrained += time_since_last_loop * pidArg->accel;
					else
						pidArg->target_dps_constrained += time_since_last_loop * pidArg->decel;

					if (pidArg->target_dps_constrained > signed_target_dps_end_decel)
						pidArg->target_dps_constrained = signed_target_dps_end_decel;
				}
				else
				{
					if (pidArg->target_dps_constrained > 0)
						pidArg->target_dps_constrained -= time_since_last_loop * pidArg->decel;
					else
						pidArg->target_dps_constrained -= time_since_last_loop * pidArg->accel;

					if (pidArg->target_dps_constrained < signed_target_dps_end_decel)
						pidArg->target_dps_constrained = signed_target_dps_end_decel;
				}

				bool old_sign_from_target_pos;
				if (position_control_enabled(pidArg))
				{
					old_sign_from_target_pos = (pidArg->x - pidArg->target_pos >= 0) ? true : false;
				}

				pidArg->x += time_since_last_loop * pidArg->target_dps_constrained;

				if (position_control_enabled(pidArg))
				{
					bool new_sign_from_target_pos = (pidArg->x - pidArg->target_pos >= 0) ? true : false;

					if (old_sign_from_target_pos != new_sign_from_target_pos)
						pidArg->x = pidArg->target_pos;
				}
			}
			else
			{
				pidArg->stalled = true;
			}

			pidArg->error = pidArg->x - pos;

			pidArg->pos_pid->setKd((1 - fabs(pidArg->target_dps_constrained) / pidArg->max_rpm / 6)
				/ time_since_last_loop_scaled * kd);

			pidArg->output = pidArg->pos_pid->compute(pidArg->error);

			//restore original gains
			pidArg->pos_pid->setKp(kp);
			pidArg->pos_pid->setKi(ki);
			pidArg->pos_pid->setKd(kd);

			runPWM_static(pidArg, pidArg->output);
		}

		else if (!motors_enabled() || !pidArg->run_pwm)
		{
			pidArg->stop_action = STOP_BRAKE;
			stopAction_static(pidArg, encoderArg, now, pos, dps);
		}
	}

	//used to attach pin change interrupts to both encoder pins
	//different ISR functions are attached depending on port (determined by pins)
	//also adds one timer interrupt shared by all ports (if not already added)
	static void attach_interrupts(encoder_state_t* encoderArg, pid_control_t* pidArg)
	{
		if (!ports_started[0] && !ports_started[1] && !ports_started[2] && !ports_started[3])
		{
			if (rp2040.cpuid == 0)
				alarm_pool_add_repeating_timer_us(EVNISRTimer0::sharedAlarmPool(), PID_TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer0::sharedISRTimer(3));
			else
				alarm_pool_add_repeating_timer_us(EVNISRTimer1::sharedAlarmPool(), PID_TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer1::sharedISRTimer(3));
		}

		switch (encoderArg->enca)
		{
		case PIN_MOTOR1_ENCA:
		case PIN_MOTOR1_ENCB:
			encoderArgs[0] = encoderArg;
			pidArgs[0] = pidArg;
			ports_started[0] = true;
			attachInterrupt(encoderArg->enca, isr0, CHANGE);
			attachInterrupt(encoderArg->encb, isr1, CHANGE);
			break;

		case PIN_MOTOR2_ENCA:
		case PIN_MOTOR2_ENCB:
			encoderArgs[1] = encoderArg;
			pidArgs[1] = pidArg;
			ports_started[1] = true;
			attachInterrupt(encoderArg->enca, isr2, CHANGE);
			attachInterrupt(encoderArg->encb, isr3, CHANGE);
			break;

		case PIN_MOTOR3_ENCA:
		case PIN_MOTOR3_ENCB:
			encoderArgs[2] = encoderArg;
			pidArgs[2] = pidArg;
			ports_started[2] = true;
			attachInterrupt(encoderArg->enca, isr4, CHANGE);
			attachInterrupt(encoderArg->encb, isr5, CHANGE);
			break;

		case PIN_MOTOR4_ENCA:
		case PIN_MOTOR4_ENCB:
			encoderArgs[3] = encoderArg;
			pidArgs[3] = pidArg;
			ports_started[3] = true;
			attachInterrupt(encoderArg->enca, isr6, CHANGE);
			attachInterrupt(encoderArg->encb, isr7, CHANGE);
			break;
		}
	}

	// pin change interrupt ISRs (calling generic functions)
	// RPM measurement only uses pulses on one encoder wheel (more accurate, from our testing)
	static void isr0() { uint64_t now = micros(); pos_update(encoderArgs[0]);velocity_update(encoderArgs[0], now); }
	static void isr1() { pos_update(encoderArgs[0]); }
	static void isr2() { uint64_t now = micros(); pos_update(encoderArgs[1]);velocity_update(encoderArgs[1], now); }
	static void isr3() { pos_update(encoderArgs[1]); }
	static void isr4() { uint64_t now = micros(); pos_update(encoderArgs[2]);velocity_update(encoderArgs[2], now); }
	static void isr5() { pos_update(encoderArgs[2]); }
	static void isr6() { uint64_t now = micros(); pos_update(encoderArgs[3]);velocity_update(encoderArgs[3], now); }
	static void isr7() { pos_update(encoderArgs[3]); }

	// timer interrupt ISR (calling generic functions for each port)
	static bool timerisr(struct repeating_timer* t)
	{
		for (int i = 0; i < 4; i++)
		{
			if (ports_started[i])
				pid_update(pidArgs[i], encoderArgs[i]);
		}
		return true;
	}
};

typedef struct
{
	//DRIVEBASE CHARACTERISTICS
	volatile uint8_t motor_type;

	volatile float max_rpm;
	volatile float max_dps;
	volatile float max_turn_rate;
	volatile float max_speed;
	volatile float wheel_dia;
	volatile float axle_track;
	EVNMotor* motor_left;
	EVNMotor* motor_right;

	volatile float speed_accel;
	volatile float speed_decel;
	volatile float turn_rate_accel;
	volatile float turn_rate_decel;

	//CONTROLLERS
	PIDController* turn_rate_pid;
	PIDController* speed_pid;

	//LOOP
	volatile uint64_t last_update;
	volatile uint8_t stop_action;

	//USER-SET
	volatile float target_speed;
	volatile float target_turn_rate;
	volatile float target_speed_constrained;
	volatile float target_turn_rate_constrained;

	volatile bool drive;
	volatile bool drive_position;
	volatile bool core0_writing;
	volatile bool stopAction_static_running;

	//DRIVE
	volatile float target_angle;
	volatile float target_distance;
	// volatile float target_position_x;
	// volatile float target_position_y;

	volatile float angle_to_target;
	volatile float angle_error;
	volatile float angle_output;

	volatile float speed_error;
	volatile float speed_output;

	volatile float target_motor_left_dps;
	volatile float target_motor_right_dps;

	//DRIVE POSITION
	// volatile float start_position_x;
	// volatile float start_position_y;
	// volatile float start_angle;
	// volatile float start_distance;
	// volatile float end_position_x;
	// volatile float end_position_y;
	volatile float end_angle;
	volatile float end_distance;

	//ODOMETRY
	volatile float current_distance;
	volatile float prev_distance;
	volatile float current_angle;
	volatile float position_x;
	volatile float position_y;
	// volatile float motor_left_x;
	// volatile float motor_left_y;
	// volatile float motor_right_x;
	// volatile float motor_right_y;
	volatile uint8_t counter;
}	drivebase_state_t;

class EVNDrivebase
{
public:
	static const uint8_t MAX_DB_OBJECTS = 2;
	static const uint16_t PID_TIMER_INTERVAL_US = 2000;

	EVNDrivebase(float wheel_dia, float axle_track, EVNMotor* motor_left, EVNMotor* motor_right);
	void begin();

	void drivePct(float speed_outer_pct, float turn_rate_pct);
	void drive(float speed, float turn_rate);
	void driveTurnRate(float speed, float turn_rate);
	void driveRadius(float speed, float radius);
	void straight(float speed, float distance, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void curve(float speed, float radius, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void curveRadius(float speed, float radius, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void curveTurnRate(float speed, float turn_rate, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void turn(float turn_rate, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void turnDegrees(float turn_rate, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void turnHeading(float turn_rate, float heading, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void driveToXY(float speed, float turn_rate, float x, float y, uint8_t stop_action = STOP_BRAKE, bool restore_initial_heading = true);

	void stop();
	void coast();
	void hold();

	bool completed();

	void setSpeedPID(float kp, float ki, float kd);
	void setTurnRatePID(float kp, float ki, float kd);
	void setSpeedAccel(float speed_accel);
	void setSpeedDecel(float speed_decel);
	void setTurnRateAccel(float turn_rate_accel);
	void setTurnRateDecel(float turn_rate_decel);

	float getDistance();
	float getAngle();
	float getHeading();
	float getX();
	float getY();
	void resetXY();
	float getDistanceToPoint(float x, float y);
	//TODO: add end function

private:
	float clean_input_turn_rate(float turn_rate);
	float clean_input_speed(float speed, float turn_rate);
	uint8_t clean_input_stop_action(uint8_t stop_action);
	float scaling_factor_for_maintaining_radius(float speed, float turn_rate);
	float radius_to_turn_rate(float speed, float radius);
	void enable_drive_position(uint8_t stop_action, bool wait);

	drivebase_state_t db;
	static drivebase_state_t* dbArgs[MAX_DB_OBJECTS];
	static bool dbs_enabled[MAX_DB_OBJECTS];

	static void attach_db_interrupt(drivebase_state_t* arg)
	{
		bool timerisr_started = false;
		for (int i = 0; i < MAX_DB_OBJECTS; i++)
		{
			if (!dbs_enabled[i])
			{
				dbArgs[i] = arg;
				dbs_enabled[i] = true;
				break;
			}
			else
				timerisr_started = true;
		}

		if (!timerisr_started)
		{
			if (rp2040.cpuid == 0)
				alarm_pool_add_repeating_timer_us(EVNISRTimer0::sharedAlarmPool(), PID_TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer0::sharedISRTimer(4));
			else
				alarm_pool_add_repeating_timer_us(EVNISRTimer1::sharedAlarmPool(), PID_TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer1::sharedISRTimer(4));
		}
	}

	static bool timerisr(struct repeating_timer* t)
	{
		for (int i = 0; i < MAX_DB_OBJECTS; i++)
		{
			if (dbs_enabled[i])
				pid_update(dbArgs[i]);
		}
		return true;
	}

	static void stopAction_static(drivebase_state_t* arg)
	{
		if (arg->core0_writing) return;
		arg->stopAction_static_running = true;

		switch (arg->stop_action)
		{
		case STOP_BRAKE:
			arg->motor_left->stop();
			arg->motor_right->stop();
			break;
		case STOP_COAST:
			arg->motor_left->coast();
			arg->motor_right->coast();
			break;
		case STOP_HOLD:
			arg->motor_left->hold();
			arg->motor_right->hold();
			break;
		}

		// we shouldn't assume drivebase speed and turn rate become 0...
		// but as of right now I have no way to get a good estimate
		arg->target_speed_constrained = 0;
		arg->target_turn_rate_constrained = 0;

		arg->target_angle = arg->current_angle;
		arg->target_distance = arg->current_distance;
		// arg->target_position_x = arg->position_x;
		// arg->target_position_y = arg->position_y;

		arg->turn_rate_pid->reset();
		arg->speed_pid->reset();

		arg->drive = false;
		arg->drive_position = false;

		arg->stopAction_static_running = false;
	}

	static float getDistance_static(drivebase_state_t* arg)
	{
		return (arg->motor_right->getPosition() + arg->motor_left->getPosition()) * arg->wheel_dia / 2 / 360 * M_PI;
	}

	static float getAngle_static(drivebase_state_t* arg)
	{
		return (arg->motor_right->getPosition() - arg->motor_left->getPosition()) * arg->wheel_dia / (2 * arg->axle_track);
	}

	static bool motors_enabled()
	{
		return ((EVNAlpha::sharedButtonLED().read() && EVNAlpha::sharedButtonLED().sharedState()->link_movement)
			|| !EVNAlpha::sharedButtonLED().sharedState()->link_movement);
	}

	static void pid_update(drivebase_state_t* arg)
	{
		uint64_t now = micros();
		float time_since_last_loop_scaled = ((float)now - (float)arg->last_update) / 1000;
		float time_since_last_loop = time_since_last_loop_scaled / 1000;
		arg->last_update = now;

		arg->current_angle = getAngle_static(arg);
		arg->current_distance = getDistance_static(arg);
		float distance_travelled_in_last_loop = arg->current_distance - arg->prev_distance;
		arg->prev_distance = arg->current_distance;

		arg->position_x += distance_travelled_in_last_loop * cos(arg->current_angle / 180 * M_PI);
		arg->position_y += distance_travelled_in_last_loop * sin(arg->current_angle / 180 * M_PI);
		// arg->motor_left_x = arg->position_x + 0.5 * arg->axle_track * cos((arg->current_angle + 90) / 180 * M_PI);
		// arg->motor_left_y = arg->position_y + 0.5 * arg->axle_track * sin((arg->current_angle + 90) / 180 * M_PI);
		// arg->motor_right_x = arg->position_x + 0.5 * arg->axle_track * cos((arg->current_angle - 90) / 180 * M_PI);
		// arg->motor_right_y = arg->position_y + 0.5 * arg->axle_track * sin((arg->current_angle - 90) / 180 * M_PI);

		if (motors_enabled() && arg->drive)
		{
			float target_speed_after_decel = arg->target_speed;
			float target_turn_rate_after_decel = arg->target_turn_rate;

			//calculation of speed & turn rate when motor is coming to a pause
			if (arg->drive_position)
			{
				float stop_speed_decel = arg->speed_decel;
				float stop_turn_rate_decel = arg->turn_rate_decel;
				float stop_time_to_decel_speed = fabs(arg->target_speed) / arg->speed_decel;
				float stop_time_to_decel_turn_rate = fabs(arg->target_turn_rate) / arg->turn_rate_decel;

				//stretch either decel to match the slower one
				if (stop_time_to_decel_speed != 0 && stop_time_to_decel_turn_rate != 0)
				{
					if (stop_time_to_decel_speed > stop_time_to_decel_turn_rate)
						stop_turn_rate_decel *= stop_time_to_decel_turn_rate / stop_time_to_decel_speed;
					else if (stop_time_to_decel_turn_rate > stop_time_to_decel_speed)
						stop_speed_decel *= stop_time_to_decel_speed / stop_time_to_decel_turn_rate;
				}

				//calculate what the current speed & turn rate should be, based on respective error
				float error_to_start_decel_speed = pow(arg->target_speed, 2) / 2 / stop_speed_decel;
				float error_to_start_decel_turn_rate = pow(arg->target_turn_rate, 2) / 2 / stop_turn_rate_decel;
				float current_distance_error = fabs(arg->end_distance - arg->current_distance);
				float current_angle_error = fabs(arg->end_angle - arg->current_angle);

				if (current_distance_error < error_to_start_decel_speed)
					target_speed_after_decel = arg->target_speed * sqrt(current_distance_error / error_to_start_decel_speed);

				if (current_angle_error < error_to_start_decel_turn_rate)
					target_turn_rate_after_decel = arg->target_turn_rate * sqrt(current_angle_error / error_to_start_decel_turn_rate);
			}

			// arg->angle_to_target = atan2(arg->target_position_y - arg->position_y, arg->target_position_x - arg->position_x);
			// arg->angle_to_target = arg->angle_to_target / M_PI * 180;
			// if (arg->angle_to_target < 0)
			// 	arg->angle_to_target += 360;
			// float error_att_to_cur = arg->angle_to_target - fmod(fmod(arg->current_angle, 360) + 360, 360);
			// if (error_att_to_cur > 180)
			//  error_att_to_cur -= 360;
			// if (error_att_to_cur < -180)
			//  error_att_to_cur += 360;
			// error_att_to_cur /= 180;
			// angle error->difference between angle to target and current angle(-1 to 1)
			// float cx = arg->target_position_x + cos(arg->target_angle / 180 * M_PI);
			// float cy = arg->target_position_y + sin(arg->target_angle / 180 * M_PI);
			// float perpendicular_distance = fabs((cx - arg->target_position_x) * (arg->position_y - arg->target_position_y) - (cy - arg->target_position_y) * (arg->position_x - arg->target_position_x));

			//angle error = the difference between the robot's current angle and the angle it should travel at
			arg->angle_error = arg->target_angle - arg->current_angle;
			if (arg->angle_error > 180)
				arg->angle_error -= 360;
			if (arg->angle_error < -180)
				arg->angle_error += 360;
			arg->angle_error /= 180;

			// float kp = arg->turn_rate_pid->getKp();
			float ki = arg->turn_rate_pid->getKi();
			// float kd = arg->turn_rate_pid->getKd();

			if (!arg->drive_position)
				arg->turn_rate_pid->setKi(0);
			else
				arg->turn_rate_pid->setKi(ki * time_since_last_loop_scaled);

			arg->angle_error = constrain(arg->angle_error, -1, 1);
			arg->angle_output = arg->turn_rate_pid->compute(arg->angle_error);

			// arg->turn_rate_pid->setKp(kp);
			arg->turn_rate_pid->setKi(ki);
			// arg->turn_rate_pid->setKd(kd);

			// kp = arg->speed_pid->getKp();
			ki = arg->speed_pid->getKi();
			// kd = arg->speed_pid->getKd();

			if (!arg->drive_position)
				arg->speed_pid->setKi(0);
			else
				arg->speed_pid->setKi(ki * time_since_last_loop_scaled);

			//speed error is the euclidean distance between the db position and its target (converted to motor degrees)
			arg->speed_error = arg->target_distance - arg->current_distance;
			arg->speed_error = arg->speed_error / M_PI / arg->wheel_dia * 360;
			arg->speed_output = arg->speed_pid->compute(arg->speed_error);

			// arg->speed_pid->setKp(kp);
			arg->speed_pid->setKi(ki);
			// arg->speed_pid->setKd(kd);

			//calculate motor speeds
			//speed output   -> average speed
			//heading output -> difference between speeds
			arg->target_motor_left_dps = arg->speed_output - arg->angle_output;
			arg->target_motor_right_dps = arg->speed_output + arg->angle_output;

			//maintain ratio between speeds when either speed exceeds motor limits
			float ratio = arg->target_motor_left_dps / arg->target_motor_right_dps;
			if (fabs(arg->target_motor_left_dps) > arg->max_dps)
			{
				if (arg->target_motor_left_dps > 0)
					arg->target_motor_left_dps = arg->max_dps;
				else
					arg->target_motor_left_dps = -arg->max_dps;
				arg->target_motor_right_dps = arg->target_motor_left_dps / ratio;
			}

			if (fabs(arg->target_motor_right_dps) > arg->max_dps)
			{
				if (arg->target_motor_right_dps > 0)
					arg->target_motor_right_dps = arg->max_dps;
				else
					arg->target_motor_right_dps = -arg->max_dps;
				arg->target_motor_left_dps = arg->target_motor_right_dps * ratio;
			}

			//write speeds to motors
			arg->motor_left->runSpeed(arg->target_motor_left_dps);
			arg->motor_right->runSpeed(arg->target_motor_right_dps);

			bool old_sign_from_target_dist;
			bool old_sign_from_target_angle;

			if (arg->drive_position)
			{
				old_sign_from_target_dist = (arg->target_distance - arg->end_distance >= 0) ? true : false;
				old_sign_from_target_angle = (arg->target_angle - arg->end_angle >= 0) ? true : false;
			}

			//increment target angle and XY position
			//if output of speed or turn rate output is saturated or motors are stalled, stop incrementing (avoid excessive overshoot that PID cannot correct)
			if (fabs(arg->speed_error * arg->speed_pid->getKp() + arg->speed_pid->getIntegral() * arg->speed_pid->getKi()) < arg->max_dps
				&& fabs(arg->angle_error * arg->turn_rate_pid->getKp() + arg->turn_rate_pid->getIntegral() * arg->turn_rate_pid->getKi()) < arg->max_dps
				&& !arg->motor_left->stalled() && !arg->motor_right->stalled()
				)
			{

				//calculating time taken to decel/accel to target speed & turn rate
				float new_speed_accel = arg->speed_accel;
				float new_speed_decel = arg->speed_decel;
				float new_turn_rate_accel = arg->turn_rate_accel;
				float new_turn_rate_decel = arg->turn_rate_decel;

				float time_to_hit_speed = 0;
				float time_to_hit_turn_rate = 0;

				if (target_speed_after_decel != arg->target_speed_constrained &&
					target_turn_rate_after_decel != arg->target_turn_rate_constrained)
				{
					if (target_speed_after_decel > 0 && arg->target_speed_constrained > 0)
					{
						float difference = target_speed_after_decel - arg->target_speed_constrained;
						if (difference > 0)
							time_to_hit_speed = fabs(difference) / arg->speed_accel;
						else
							time_to_hit_speed = fabs(difference) / arg->speed_decel;
					}
					else if (target_speed_after_decel < 0 && arg->target_speed_constrained < 0)
					{
						float difference = target_speed_after_decel - arg->target_speed_constrained;
						if (difference < 0)
							time_to_hit_speed = fabs(difference) / arg->speed_accel;
						else
							time_to_hit_speed = fabs(difference) / arg->speed_decel;
					}
					else
						time_to_hit_speed = fabs(arg->target_speed_constrained) / arg->speed_decel +
						fabs(target_speed_after_decel) / arg->speed_accel;

					if (target_turn_rate_after_decel > 0 && arg->target_turn_rate_constrained > 0)
					{
						float difference = target_turn_rate_after_decel - arg->target_turn_rate_constrained;
						if (difference > 0)
							time_to_hit_turn_rate = fabs(difference) / arg->turn_rate_accel;
						else
							time_to_hit_turn_rate = fabs(difference) / arg->turn_rate_decel;
					}
					else if (target_turn_rate_after_decel < 0 && arg->target_turn_rate_constrained < 0)
					{
						float difference = target_turn_rate_after_decel - arg->target_turn_rate_constrained;
						if (difference < 0)
							time_to_hit_turn_rate = fabs(difference) / arg->turn_rate_accel;
						else
							time_to_hit_turn_rate = fabs(difference) / arg->turn_rate_decel;
					}
					else
						time_to_hit_turn_rate = fabs(arg->target_turn_rate_constrained) / arg->turn_rate_decel +
						fabs(target_turn_rate_after_decel) / arg->turn_rate_accel;

					//stretch the accel/decel trajectories for turn rate and speed to match each other
					if (time_to_hit_speed != 0 && time_to_hit_turn_rate != 0)
					{
						if (time_to_hit_speed > time_to_hit_turn_rate)
						{
							//slow down turn rate to match time taken to hit target speed
							new_turn_rate_accel *= constrain(time_to_hit_turn_rate / time_to_hit_speed, 0, 1);
							new_turn_rate_decel *= constrain(time_to_hit_turn_rate / time_to_hit_speed, 0, 1);
						}
						else
						{
							//slow down speed to match time taken to hit target turn_rate
							new_speed_accel *= constrain(time_to_hit_speed / time_to_hit_turn_rate, 0, 1);
							new_speed_decel *= constrain(time_to_hit_speed / time_to_hit_turn_rate, 0, 1);
						}
					}
				}

				//apply accel/decel to hit target speed
				if (arg->target_speed_constrained < target_speed_after_decel)
				{
					if (arg->target_speed_constrained > 0)
						arg->target_speed_constrained += time_since_last_loop * new_speed_accel;
					else
						arg->target_speed_constrained += time_since_last_loop * new_speed_decel;

					if (arg->target_speed_constrained > target_speed_after_decel)
						arg->target_speed_constrained = target_speed_after_decel;
				}
				else if (arg->target_speed_constrained > target_speed_after_decel)
				{
					if (arg->target_speed_constrained > 0)
						arg->target_speed_constrained -= time_since_last_loop * new_speed_decel;
					else
						arg->target_speed_constrained -= time_since_last_loop * new_speed_accel;

					if (arg->target_speed_constrained < target_speed_after_decel)
						arg->target_speed_constrained = target_speed_after_decel;
				}

				//apply accel/decel to hit target turn rate
				if (arg->target_turn_rate_constrained < target_turn_rate_after_decel)
				{
					if (arg->target_turn_rate_constrained > 0)
						arg->target_turn_rate_constrained += time_since_last_loop * new_turn_rate_accel;
					else
						arg->target_turn_rate_constrained += time_since_last_loop * new_turn_rate_decel;

					if (arg->target_turn_rate_constrained > target_turn_rate_after_decel)
						arg->target_turn_rate_constrained = target_turn_rate_after_decel;
				}
				else if (arg->target_turn_rate_constrained > target_turn_rate_after_decel)
				{
					if (arg->target_turn_rate_constrained > 0)
						arg->target_turn_rate_constrained -= time_since_last_loop * new_turn_rate_decel;
					else
						arg->target_turn_rate_constrained -= time_since_last_loop * new_turn_rate_accel;

					if (arg->target_turn_rate_constrained < target_turn_rate_after_decel)
						arg->target_turn_rate_constrained = target_turn_rate_after_decel;
				}

				//increment target angle and distance
				arg->target_angle += time_since_last_loop * arg->target_turn_rate_constrained;
				arg->target_distance += time_since_last_loop * arg->target_speed_constrained;
				// arg->target_position_x += time_since_last_loop * arg->target_speed * cos(arg->target_angle / 180 * M_PI) * (1 - fabs(arg->angle_output));
				// arg->target_position_y += time_since_last_loop * arg->target_speed * sin(arg->target_angle / 180 * M_PI) * (1 - fabs(arg->angle_output));
			}

			if (arg->drive_position)
			{
				bool new_sign_from_target_dist = (arg->target_distance - arg->end_distance >= 0) ? true : false;
				bool new_sign_from_target_angle = (arg->target_angle - arg->end_angle >= 0) ? true : false;

				if (old_sign_from_target_dist != new_sign_from_target_dist)
					arg->target_distance = arg->end_distance;

				if (old_sign_from_target_angle != new_sign_from_target_angle)
					arg->target_angle = arg->end_angle;

				if (fabs(arg->end_angle - arg->current_angle) <= USER_DRIVE_POS_MIN_ERROR_DEG
					&& fabs(arg->end_distance - arg->current_distance) <= USER_DRIVE_POS_MIN_ERROR_MM)
					arg->counter++;
				else
					arg->counter = 0;

				if (arg->counter >= USER_DRIVE_POS_MIN_LOOP_COUNT)
					stopAction_static(arg);
			}
		}
		else
		{
			arg->stop_action = STOP_BRAKE;
			stopAction_static(arg);
		}
	}
};

class EVNOmniDrivebaseBasic
{
public:
	EVNOmniDrivebaseBasic(uint32_t wheel_dia, uint32_t wheel_dist, EVNMotor* fl, EVNMotor* fr, EVNMotor* bl, EVNMotor* br)
	{
		_fl = fl;
		_fr = fr;
		_bl = bl;
		_br = br;
	};

	void begin()
	{
		_max_rpm = min(min(_fl->_pid_control.max_rpm, _fr->_pid_control.max_rpm), min(_bl->_pid_control.max_rpm, _br->_pid_control.max_rpm));
	};

	void steer(float speed, float angle, float turn_rate)
	{
		float speedc, anglec, rotatec;
		float anglec_rad, speedc_x, speedc_y;
		float speedc_x_left, speedc_y_left, speedc_x_right, speedc_y_right;

		speedc = constrain(speed, -_max_rpm * 6, _max_rpm * 6);
		anglec = constrain(angle, 0, 360);
		anglec = fmod(anglec + 45, 360);
		rotatec = constrain(turn_rate, -1, 1);

		anglec_rad = anglec / 180 * M_PI;
		speedc_x = sin(anglec_rad) * speedc;
		speedc_y = cos(anglec_rad) * speedc;

		if (anglec >= 90 && anglec <= 270)
		{
			if (rotatec >= 0)
			{
				speedc_y_left = speedc_y * (1 - 2 * rotatec);
				speedc_y_right = speedc_y;
			}
			else {
				speedc_y_right = speedc_y * (1 + 2 * rotatec);
				speedc_y_left = speedc_y;
			}
		}
		else {
			if (rotatec >= 0)
			{
				speedc_y_right = speedc_y * (1 - 2 * rotatec);
				speedc_y_left = speedc_y;
			}
			else {
				speedc_y_left = speedc_y * (1 + 2 * rotatec);
				speedc_y_right = speedc_y;
			}
		}

		if (anglec >= 0 && anglec <= 180)
		{
			if (rotatec >= 0)
			{
				speedc_x_right = speedc_x * (1 - 2 * rotatec);
				speedc_x_left = speedc_x;
			}
			else {
				speedc_x_left = speedc_x * (1 + 2 * rotatec);
				speedc_x_right = speedc_x;
			}
		}
		else {
			if (rotatec >= 0)
			{
				speedc_x_left = speedc_x * (1 - 2 * rotatec);
				speedc_x_right = speedc_x;
			}
			else {
				speedc_x_right = speedc_x * (1 + 2 * rotatec);
				speedc_x_left = speedc_x;
			}
		}

		_fl->runSpeed(speedc_x_left);
		_br->runSpeed(speedc_x_right);
		_fr->runSpeed(speedc_y_right);
		_bl->runSpeed(speedc_y_left);
	};

	void stop()
	{
		_fl->stop();
		_fr->stop();
		_bl->stop();
		_br->stop();
	};

	void coast()
	{
		_fl->coast();
		_fr->coast();
		_bl->coast();
		_br->coast();
	};

	void hold()
	{
		_fl->hold();
		_fr->hold();
		_bl->hold();
		_br->hold();
	};

private:
	EVNMotor* _fl;
	EVNMotor* _fr;
	EVNMotor* _bl;
	EVNMotor* _br;
	float _max_rpm;
};

#endif
