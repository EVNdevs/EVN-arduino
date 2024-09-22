#ifndef EVNMotor_h
#define EVNMotor_h

#include <Arduino.h>
#include "../EVNAlpha.h"
#include "../helper/EVNISRTimer.h"
#include "../helper/PIDController.h"
#include "../evn_motor_defs.h"
#include "../evn_pins_defs.h"

// TODO: Add end function for classes
// TODO: Wrap mutex & spinlock in helper class

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

class EVNMotor;

typedef struct
{
	//POSITION MEASUREMENT
	uint8_t enca;
	uint8_t encb;
	bool enca_state;
	bool encb_state;
	int8_t dir;
	uint8_t state;
	float ppr;
	float position;
	float position_offset;

	//DPS MEASUREMENT (TIME BETWEEN PULSES)
	uint8_t last_edge_index;
	uint32_t edge_times[NO_OF_EDGES_STORED];
	bool dps_calculated;
	bool obtained_one_pulse;
	float avg_dps;
	float avg_pulse_width;
} encoder_state_t;

typedef struct
{
	//MOTOR CHARACTERISTICS
	uint8_t motor_type;
	uint8_t motora;
	uint8_t motorb;
	float max_rpm;
	float accel;
	float decel;

	//CONTROLLER
	PIDController* pos_pid;

	//USER-SET VARIABLES
	bool run_pwm;
	bool run_speed;
	bool run_dir;
	float target_dps;
	bool run_pos;
	float target_pos;
	bool run_time;
	uint32_t run_time_ms;
	bool hold;
	uint8_t stop_action;

	//LOOP VARIABLES
	uint32_t last_update;
	float target_dps_end_decel;
	float target_dps_constrained;
	float x;
	float error;
	float output;
	uint8_t counter;
	uint32_t start_time_us;
	bool stalled;
} pid_control_t;

typedef struct
{
	//DRIVEBASE CHARACTERISTICS
	uint8_t motor_type;
	float max_rpm;
	float max_dps;
	float max_turn_rate;
	float max_speed;
	float wheel_dia;
	float axle_track;
	EVNMotor* motor_left;
	EVNMotor* motor_right;

	float speed_accel;
	float speed_decel;
	float turn_rate_accel;
	float turn_rate_decel;

	//CONTROLLERS
	PIDController* turn_rate_pid;
	PIDController* speed_pid;

	//USER-SET
	float target_speed;
	float target_turn_rate;
	float target_speed_constrained;
	float target_turn_rate_constrained;
	bool drive;
	bool drive_position;

	//LOOP
	uint32_t last_update;
	uint8_t stop_action;
	float target_angle;
	float target_distance;
	// float target_position_x;
	// float target_position_y;
	float angle_to_target;
	float angle_error;
	float angle_output;
	float speed_error;
	float speed_output;
	float target_motor_left_dps;
	float target_motor_right_dps;
	float end_angle;
	float end_distance;
	float current_distance;
	float prev_distance;
	float current_angle;
	float position_x;
	float position_y;
	// float motor_left_x;
	// float motor_left_y;
	// float motor_right_x;
	// float motor_right_y;
	uint8_t counter;

} drivebase_state_t;

class EVNMotor
{
public:
	static const uint32_t PWM_FREQ = 20000;
	static const uint32_t PWM_MAX_VAL = 255;
	static const uint16_t PID_TIMER_INTERVAL_US = 2500;
	static const uint32_t ENCODER_PULSE_TIMEOUT_US = 166667 * 2;
	static const uint8_t MAX_MOTOR_OBJECTS = 4;

	friend class EVNDrivebase;
	friend class EVNOmniDrivebaseBasic;

	EVNMotor(uint8_t port, uint8_t motortype = EV3_LARGE, uint8_t motor_dir = DIRECT, uint8_t enc_dir = DIRECT);
	void begin() volatile;
	float getPosition() volatile;
	float getHeading() volatile;
	void setPosition(float position) volatile;
	void resetPosition() volatile;
	float getSpeed() volatile;

	void runPWM(float duty_cycle_pct) volatile;
	void runSpeed(float dps) volatile;
	void runPosition(float dps, float position, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void runAngle(float dps, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void runHeading(float dps, float heading, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void runTime(float dps, uint32_t time_ms, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;

	void stop() volatile;
	void coast() volatile;
	void hold() volatile;

	bool completed() volatile;
	bool stalled() volatile;

	void setPID(float p, float i, float d) volatile;
	void setAccel(float accel_dps_sq) volatile;
	void setDecel(float decel_dps_sq) volatile;
	void setMaxRPM(float max_rpm) volatile;
	void setPPR(uint32_t ppr) volatile;

protected:
	void ensure_isr_executed() volatile;

	void runSpeed_unsafe(float dps) volatile;
	void stop_unsafe() volatile;
	void coast_unsafe() volatile;
	void hold_unsafe() volatile;
	bool stalled_unsafe() volatile;

	float clean_input_dps(float dps) volatile;
	uint8_t clean_input_dir(float dps) volatile;
	uint8_t clean_input_stop_action(uint8_t stop_action) volatile;

	volatile pid_control_t _pid_control = {};
	volatile encoder_state_t _encoder = {};

	static volatile encoder_state_t* encoderArgs[MAX_MOTOR_OBJECTS];
	static volatile pid_control_t* pidArgs[MAX_MOTOR_OBJECTS];
	static volatile bool ports_started[MAX_MOTOR_OBJECTS];
	static volatile bool timerisr_enabled;
	static volatile bool timerisr_executed;
	static volatile uint8_t core;
	static mutex_t mutex;
	static spin_lock_t* spin_lock;

	static bool timed_control_enabled(volatile pid_control_t* arg)
	{
		return arg->run_time;
	}

	static bool position_control_enabled(volatile pid_control_t* arg)
	{
		return arg->run_pos || arg->hold;
	}

	static bool loop_control_enabled(volatile pid_control_t* arg)
	{
		return (arg->run_speed || arg->run_time || arg->run_pos || arg->hold);
	}

	static void velocity_update(volatile encoder_state_t* arg, uint32_t now)
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

	static void pos_update(volatile encoder_state_t* arg)
	{
		arg->enca_state = digitalRead(arg->enca);
		arg->encb_state = digitalRead(arg->encb);

		uint8_t state = arg->state & 3;

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

	static void runPWM_static(volatile pid_control_t* pidArg, float speed)
	{
		float speedc = constrain(speed, -1, 1);

		if (EVNAlpha::motorsEnabled()) {
			if (speedc > 0)
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

	static void stopAction_static(volatile pid_control_t* pidArg, volatile encoder_state_t* encoderArg, float pos, float dps)
	{
		//keep at most recent state
		pidArg->target_dps_constrained = dps;

		//coast and brake stop functions based on DRV8833 datasheet
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
	}

	static float getPosition_static(volatile encoder_state_t* arg)
	{
		// resolution of encoder readout is in CPR (4 * PPR)
		return ((float)arg->position * 90.0 / arg->ppr) - arg->position_offset;
	}

	static float getDPS_static(volatile encoder_state_t* arg)
	{
		int64_t last_pulse_width = micros() - arg->edge_times[arg->last_edge_index];
		int64_t last_full_pulse_width = 0;

		if (!arg->obtained_one_pulse) return 0;

		if (!arg->dps_calculated)
		{
			float sum_dps = 0;
			uint8_t number_of_pulses = 0;

			for (uint8_t i = 0; i < NO_OF_EDGES_STORED; i++)
			{
				uint8_t end_index = (arg->last_edge_index - i + NO_OF_EDGES_STORED) % NO_OF_EDGES_STORED;
				uint8_t start_index = (end_index - 1 + NO_OF_EDGES_STORED) % NO_OF_EDGES_STORED;
				uint32_t end = arg->edge_times[end_index];
				uint32_t start = arg->edge_times[start_index];

				int64_t pulse_width = end - start;
				float pulse_dps = 0;
				if (pulse_width > 0)
				{
					pulse_dps = 360000000.0 / pulse_width / arg->ppr;
					number_of_pulses++;
				}

				if (i == NO_OF_EDGES_STORED - 1)
					last_full_pulse_width = pulse_width;

				sum_dps += pulse_dps;
			}

			arg->avg_dps = sum_dps / number_of_pulses;
			arg->dps_calculated = true;
		}

		// if timeout, DPS is 0
		if (last_pulse_width > ENCODER_PULSE_TIMEOUT_US) return 0;

		// if latest pulse is longer than pulse width of last 2 edges, use latest pulse
		// this occurs when the motor is slowing down (pulses get longer and longer)
		else if (last_pulse_width > last_full_pulse_width)
		{
			float final_dps = 360000000.0 / last_pulse_width / arg->ppr;
			return final_dps * (float)arg->dir;
		}
		else
			return arg->avg_dps * (float)arg->dir;
	}

	static void pid_update(volatile pid_control_t* pidArg, volatile encoder_state_t* encoderArg)
	{
		uint32_t now = micros();
		float pos = getPosition_static(encoderArg);
		float dps = getDPS_static(encoderArg);
		float time_since_last_loop_scaled = ((float)(now - pidArg->last_update)) / 1000;	//milliseconds
		float time_since_last_loop = time_since_last_loop_scaled / 1000;					//seconds
		pidArg->last_update = now;

		if (time_since_last_loop < 0)
			return;

		if (EVNAlpha::motorsEnabled() && loop_control_enabled(pidArg))
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
						stopAction_static(pidArg, encoderArg, pos, dps);
						return;
					}
				}

				pidArg->run_dir = (pidArg->target_pos - pos > 0) ? DIRECT : REVERSE;

				float error = fabs(pidArg->target_pos - pos);
				float max_error_before_decel = pow(fabs(pidArg->target_dps), 2) / pidArg->decel / 2;

				if (error < max_error_before_decel)
					decel_dps = sqrt(error / max_error_before_decel) * fabs(pidArg->target_dps);
			}

			if (timed_control_enabled(pidArg))
			{
				if ((now - pidArg->start_time_us) >= pidArg->run_time_ms * 1000)
				{
					stopAction_static(pidArg, encoderArg, pos, dps);
					return;
				}

				float run_time_s = (float)pidArg->run_time_ms / 1000;
				float elapsed_run_time_s = ((float)now - (float)pidArg->start_time_us) / 1000000;
				if (pidArg->target_dps / pidArg->decel > run_time_s - elapsed_run_time_s)
					decel_dps = pidArg->decel * (run_time_s - elapsed_run_time_s);
			}

			pidArg->target_dps_end_decel = decel_dps;
			float signed_target_dps_end_decel = pidArg->target_dps_end_decel * ((pidArg->run_dir == DIRECT) ? 1 : -1);

			float kp = pidArg->pos_pid->getKp();
			float ki = pidArg->pos_pid->getKi();
			float kd = pidArg->pos_pid->getKd();

			if (!position_control_enabled(pidArg) || pidArg->stalled)
			{
				pidArg->pos_pid->setKi(0);
				pidArg->pos_pid->resetIntegral();
			}
			else
				pidArg->pos_pid->setKi(ki * time_since_last_loop_scaled);

			if (fabs((pidArg->x - pos) * pidArg->pos_pid->getKp() + pidArg->pos_pid->getIntegral() * pidArg->pos_pid->getKi()) <= 1) //anti-windup
			{
				pidArg->stalled = false;

				if (pidArg->target_dps_constrained < signed_target_dps_end_decel)
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
				bool no_change_to_target_pos = false;

				if (position_control_enabled(pidArg))
				{
					old_sign_from_target_pos = pidArg->x - pidArg->target_pos > 0;
					no_change_to_target_pos = pidArg->x == pidArg->target_pos;
				}

				if (!no_change_to_target_pos)
					pidArg->x += time_since_last_loop * pidArg->target_dps_constrained;

				if (position_control_enabled(pidArg))
				{
					bool new_sign_from_target_pos = pidArg->x - pidArg->target_pos > 0;

					if (old_sign_from_target_pos != new_sign_from_target_pos || no_change_to_target_pos)
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

		else if (!EVNAlpha::motorsEnabled() || !pidArg->run_pwm)
		{
			pidArg->stop_action = STOP_BRAKE;
			stopAction_static(pidArg, encoderArg, pos, dps);
		}
	}

	//different pin change interrupts attached depending on port (determined by pins)
	//also adds timer interrupt shared by all ports (if not already added)
	static void attach_interrupts(volatile encoder_state_t* encoderArg, volatile pid_control_t* pidArg)
	{
		switch (encoderArg->enca)
		{
		case PIN_MOTOR1_ENCA:
		case PIN_MOTOR1_ENCB:
			if (!ports_started[0])
			{
				encoderArgs[0] = encoderArg;
				pidArgs[0] = pidArg;
				ports_started[0] = true;
				attachInterrupt(encoderArg->enca, isr0, CHANGE);
				attachInterrupt(encoderArg->encb, isr1, CHANGE);
			}
			break;

		case PIN_MOTOR2_ENCA:
		case PIN_MOTOR2_ENCB:
			if (!ports_started[1])
			{
				encoderArgs[1] = encoderArg;
				pidArgs[1] = pidArg;
				ports_started[1] = true;
				attachInterrupt(encoderArg->enca, isr2, CHANGE);
				attachInterrupt(encoderArg->encb, isr3, CHANGE);
			}
			break;

		case PIN_MOTOR3_ENCA:
		case PIN_MOTOR3_ENCB:
			if (!ports_started[2])
			{
				encoderArgs[2] = encoderArg;
				pidArgs[2] = pidArg;
				ports_started[2] = true;
				attachInterrupt(encoderArg->enca, isr4, CHANGE);
				attachInterrupt(encoderArg->encb, isr5, CHANGE);
			}
			break;

		case PIN_MOTOR4_ENCA:
		case PIN_MOTOR4_ENCB:
			if (!ports_started[3])
			{
				encoderArgs[3] = encoderArg;
				pidArgs[3] = pidArg;
				ports_started[3] = true;
				attachInterrupt(encoderArg->enca, isr6, CHANGE);
				attachInterrupt(encoderArg->encb, isr7, CHANGE);
			}
			break;
		}

		if (!timerisr_enabled)
		{
			mutex_init(&mutex);
			spin_lock = spin_lock_init(spin_lock_claim_unused(true));

			core = rp2040.cpuid();
			if (core == 0)
				alarm_pool_add_repeating_timer_us(EVNISRTimer0::sharedAlarmPool(), PID_TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer0::sharedISRTimer(3));
			else
				alarm_pool_add_repeating_timer_us(EVNISRTimer1::sharedAlarmPool(), PID_TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer1::sharedISRTimer(3));

			timerisr_enabled = true;
		}
	}

	// pin change interrupt ISRs (calling generic functions)
	// RPM measurement only uses pulses on one encoder wheel (more accurate, from our testing)
	static void isr0()
	{
		uint32_t now = micros();
		if (mutex_try_enter_block_until(&mutex, delayed_by_us(get_absolute_time(), 1000)))
		{
			pos_update(encoderArgs[0]);
			velocity_update(encoderArgs[0], now);
			mutex_exit(&mutex);
		}
	}

	static void isr1()
	{
		if (mutex_try_enter_block_until(&mutex, delayed_by_us(get_absolute_time(), 1000)))
		{
			pos_update(encoderArgs[0]);
			mutex_exit(&mutex);
		}
	}

	static void isr2()
	{
		uint32_t now = micros();
		if (mutex_try_enter_block_until(&mutex, delayed_by_us(get_absolute_time(), 1000)))
		{
			pos_update(encoderArgs[1]);
			velocity_update(encoderArgs[1], now);
			mutex_exit(&mutex);
		}
	}

	static void isr3()
	{
		if (mutex_try_enter_block_until(&mutex, delayed_by_us(get_absolute_time(), 1000)))
		{
			pos_update(encoderArgs[1]);
			mutex_exit(&mutex);
		}
	}

	static void isr4()
	{
		uint32_t now = micros();
		if (mutex_try_enter_block_until(&mutex, delayed_by_us(get_absolute_time(), 1000)))
		{
			pos_update(encoderArgs[2]);
			velocity_update(encoderArgs[2], now);
			mutex_exit(&mutex);
		}
	}

	static void isr5()
	{
		if (mutex_try_enter_block_until(&mutex, delayed_by_us(get_absolute_time(), 1000)))
		{
			pos_update(encoderArgs[2]);
			mutex_exit(&mutex);
		}
	}

	static void isr6()
	{
		uint32_t now = micros();
		if (mutex_try_enter_block_until(&mutex, delayed_by_us(get_absolute_time(), 1000)))
		{
			pos_update(encoderArgs[3]);
			velocity_update(encoderArgs[3], now);
			mutex_exit(&mutex);
		}
	}

	static void isr7()
	{
		if (mutex_try_enter_block_until(&mutex, delayed_by_us(get_absolute_time(), 1000)))
		{
			pos_update(encoderArgs[3]);
			mutex_exit(&mutex);
		}
	}

	// timer interrupt ISR
	static bool timerisr(struct repeating_timer* t)
	{
		if (mutex_try_enter_block_until(&mutex, delayed_by_us(get_absolute_time(), 1000)))
		{
			spin_lock_unsafe_blocking(spin_lock);

			for (int i = 0; i < MAX_MOTOR_OBJECTS; i++)
			{
				if (ports_started[i])
					pid_update(pidArgs[i], encoderArgs[i]);
			}

			timerisr_executed = true;
			spin_unlock_unsafe(spin_lock);
			mutex_exit(&mutex);
		}

		return true;
	}
};

class EVNDrivebase
{
public:
	static const uint8_t MAX_DB_OBJECTS = 2;
	static const uint16_t PID_TIMER_INTERVAL_US = 2500;

	EVNDrivebase(float wheel_dia, float axle_track, EVNMotor* motor_left, EVNMotor* motor_right);
	void begin() volatile;

	void drivePct(float speed_outer_pct, float turn_rate_pct) volatile;
	void drive(float speed, float turn_rate) volatile;
	void driveTurnRate(float speed, float turn_rate) volatile;
	void driveRadius(float speed, float radius) volatile;
	void straight(float speed, float distance, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void curve(float speed, float radius, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void curveRadius(float speed, float radius, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void curveTurnRate(float speed, float turn_rate, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void turn(float turn_rate, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void turnDegrees(float turn_rate, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void turnHeading(float turn_rate, float heading, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void driveToXY(float speed, float turn_rate, float x, float y, uint8_t stop_action = STOP_BRAKE, bool restore_initial_heading = true) volatile;

	void stop() volatile;
	void coast() volatile;
	void hold() volatile;
	bool completed() volatile;

	void setSpeedPID(float kp, float ki, float kd) volatile;
	void setTurnRatePID(float kp, float ki, float kd) volatile;
	void setSpeedAccel(float speed_accel) volatile;
	void setSpeedDecel(float speed_decel) volatile;
	void setTurnRateAccel(float turn_rate_accel) volatile;
	void setTurnRateDecel(float turn_rate_decel) volatile;

	float getDistance() volatile;
	float getAngle() volatile;
	float getHeading() volatile;
	float getX() volatile;
	float getY() volatile;
	void resetXY() volatile;
	float getDistanceToPoint(float x, float y) volatile;

private:
	void ensure_isr_executed() volatile;

	float clean_input_turn_rate(float turn_rate) volatile;
	float clean_input_speed(float speed, float turn_rate) volatile;
	uint8_t clean_input_stop_action(uint8_t stop_action) volatile;
	float scaling_factor_for_maintaining_radius(float speed, float turn_rate) volatile;
	float radius_to_turn_rate(float speed, float radius) volatile;

	volatile drivebase_state_t db = {};
	static volatile drivebase_state_t* dbArgs[MAX_DB_OBJECTS];
	static volatile bool dbs_started[MAX_DB_OBJECTS];
	static volatile bool timerisr_enabled;
	static volatile bool timerisr_executed;
	static volatile uint8_t core;

	static void attach_db_interrupt(volatile drivebase_state_t* arg)
	{
		for (int i = 0; i < MAX_DB_OBJECTS; i++)
		{
			if (!dbs_started[i])
			{
				dbArgs[i] = arg;
				dbs_started[i] = true;
				break;
			}
		}

		if (!timerisr_enabled)
		{
			if (EVNMotor::timerisr_enabled)
			{
				//remove timer interrupt added by EVNMotor (if added)
				cancel_repeating_timer(&EVNISRTimer0::sharedISRTimer(3));
				cancel_repeating_timer(&EVNISRTimer1::sharedISRTimer(3));
			}
			else
			{
				//initialize mutex & spin lock otherwise
				mutex_init(&EVNMotor::mutex);
				EVNMotor::spin_lock = spin_lock_init(spin_lock_claim_unused(true));
			}

			core = rp2040.cpuid();
			if (core == 0)
				alarm_pool_add_repeating_timer_us(EVNISRTimer0::sharedAlarmPool(), PID_TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer0::sharedISRTimer(4));
			else
				alarm_pool_add_repeating_timer_us(EVNISRTimer1::sharedAlarmPool(), PID_TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer1::sharedISRTimer(4));

			EVNMotor::timerisr_enabled = true;
			timerisr_enabled = true;
		}
	}

	static bool timerisr(struct repeating_timer* t)
	{
		if (mutex_try_enter_block_until(&EVNMotor::mutex, delayed_by_us(get_absolute_time(), 1000)))
		{
			spin_lock_unsafe_blocking(EVNMotor::spin_lock);

			for (int i = 0; i < MAX_DB_OBJECTS; i++)
			{
				if (dbs_started[i])
					pid_update(dbArgs[i]);
			}

			for (int i = 0; i < EVNMotor::MAX_MOTOR_OBJECTS; i++)
			{
				if (EVNMotor::ports_started[i])
					EVNMotor::pid_update(EVNMotor::pidArgs[i], EVNMotor::encoderArgs[i]);
			}

			timerisr_executed = true;
			spin_unlock_unsafe(EVNMotor::spin_lock);
			mutex_exit(&EVNMotor::mutex);
		}
		return true;
	}

	static void stopAction_static(volatile drivebase_state_t* arg)
	{
		switch (arg->stop_action)
		{
		case STOP_BRAKE:
			arg->motor_left->stop_unsafe();
			arg->motor_right->stop_unsafe();
			break;
		case STOP_COAST:
			arg->motor_left->coast_unsafe();
			arg->motor_right->coast_unsafe();
			break;
		case STOP_HOLD:
			arg->motor_left->hold_unsafe();
			arg->motor_right->hold_unsafe();
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
	}

	static float getDistance_static(volatile drivebase_state_t* arg)
	{
		return (arg->motor_right->getPosition_static(&arg->motor_right->_encoder) + arg->motor_left->getPosition_static(&arg->motor_left->_encoder)) * arg->wheel_dia / 2 / 360 * M_PI;
	}

	static float getAngle_static(volatile drivebase_state_t* arg)
	{
		return (arg->motor_right->getPosition_static(&arg->motor_right->_encoder) - arg->motor_left->getPosition_static(&arg->motor_left->_encoder)) * arg->wheel_dia / (2 * arg->axle_track);
	}

	static void pid_update(volatile drivebase_state_t* arg)
	{
		//update time between loops
		uint32_t now = micros();
		float time_since_last_loop_scaled = ((float)now - (float)arg->last_update) / 1000;
		float time_since_last_loop = time_since_last_loop_scaled / 1000;
		arg->last_update = now;

		if (time_since_last_loop < 0)
			return;

		//update angle and linear distance travelled
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

		if (EVNAlpha::motorsEnabled() && arg->drive)
		{
			float target_speed_after_decel = arg->target_speed;
			float target_turn_rate_after_decel = arg->target_turn_rate;

			//calculate speed & turn rate when motor is coming to a pause
			if (arg->drive_position)
			{
				float stop_speed_decel = arg->speed_decel;
				float stop_turn_rate_decel = arg->turn_rate_decel;
				float stop_time_to_decel_speed = fabs(arg->target_speed) / arg->speed_decel;
				float stop_time_to_decel_turn_rate = fabs(arg->target_turn_rate) / arg->turn_rate_decel;

				//slow down either decel to match each each other
				if (stop_time_to_decel_speed != 0 && stop_time_to_decel_turn_rate != 0)
				{
					if (stop_time_to_decel_speed > stop_time_to_decel_turn_rate)
						stop_turn_rate_decel *= stop_time_to_decel_turn_rate / stop_time_to_decel_speed;
					else if (stop_time_to_decel_turn_rate > stop_time_to_decel_speed)
						stop_speed_decel *= stop_time_to_decel_speed / stop_time_to_decel_turn_rate;
				}

				//calculate what the current max speed & turn rate should be, based on respective errors
				float error_to_start_decel_speed = pow(arg->target_speed, 2) / 2 / stop_speed_decel;
				float error_to_start_decel_turn_rate = pow(arg->target_turn_rate, 2) / 2 / stop_turn_rate_decel;
				float current_distance_error = fabs(arg->end_distance - arg->current_distance);
				float current_angle_error = fabs(arg->end_angle - arg->current_angle);

				if (current_distance_error < error_to_start_decel_speed)
					target_speed_after_decel = arg->target_speed * sqrt(current_distance_error / error_to_start_decel_speed);

				if (current_angle_error < error_to_start_decel_turn_rate)
					target_turn_rate_after_decel = arg->target_turn_rate * sqrt(current_angle_error / error_to_start_decel_turn_rate);
			}

			//angle error -> difference between the robot's current angle and the angle it should travel at
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
			{
				arg->turn_rate_pid->setKi(0);
				arg->turn_rate_pid->resetIntegral();
			}
			else
				arg->turn_rate_pid->setKi(ki * time_since_last_loop_scaled);

			arg->angle_output = arg->turn_rate_pid->compute(arg->angle_error);

			// arg->turn_rate_pid->setKp(kp);
			arg->turn_rate_pid->setKi(ki);
			// arg->turn_rate_pid->setKd(kd);

			// kp = arg->speed_pid->getKp();
			ki = arg->speed_pid->getKi();
			// kd = arg->speed_pid->getKd();

			if (!arg->drive_position)
			{
				arg->speed_pid->setKi(0);
				arg->speed_pid->resetIntegral();
			}
			else
				arg->speed_pid->setKi(ki * time_since_last_loop_scaled);

			//speed error -> distance between the db position and its target (converted to motor degrees)
			arg->speed_error = arg->target_distance - arg->current_distance;
			arg->speed_error = arg->speed_error / M_PI / arg->wheel_dia * 360;
			arg->speed_output = arg->speed_pid->compute(arg->speed_error);

			// arg->speed_pid->setKp(kp);
			arg->speed_pid->setKi(ki);
			// arg->speed_pid->setKd(kd);

			//calculate motor speeds
			//speed PID output   -> average speed
			//angle PID output -> difference between speeds
			arg->target_motor_left_dps = arg->speed_output - arg->angle_output;
			arg->target_motor_right_dps = arg->speed_output + arg->angle_output;

			//maintain ratio between speeds when either exceeds motor limits
			if (arg->target_motor_left_dps != 0 && arg->target_motor_right_dps != 0)
			{
				if (fabs(arg->target_motor_left_dps) > arg->max_dps)
				{
					float ratio = arg->target_motor_right_dps / arg->target_motor_left_dps;
					if (arg->target_motor_left_dps > 0)
						arg->target_motor_left_dps = arg->max_dps;
					else
						arg->target_motor_left_dps = -arg->max_dps;
					arg->target_motor_right_dps = arg->target_motor_left_dps * ratio;
				}

				if (fabs(arg->target_motor_right_dps) > arg->max_dps)
				{
					float ratio = arg->target_motor_left_dps / arg->target_motor_right_dps;
					if (arg->target_motor_right_dps > 0)
						arg->target_motor_right_dps = arg->max_dps;
					else
						arg->target_motor_right_dps = -arg->max_dps;
					arg->target_motor_left_dps = arg->target_motor_right_dps * ratio;
				}
			}

			//write speeds to motors
			//non-thread safe write is used here
			//assumed safe because EVNDrivebase and EVNMotor should be run on same core
			arg->motor_left->runSpeed_unsafe(arg->target_motor_left_dps);
			arg->motor_right->runSpeed_unsafe(arg->target_motor_right_dps);

			//preserve sign of error between target and end distance/angle
			bool old_sign_from_target_distance;
			bool old_sign_from_target_angle;
			bool no_change_to_target_distance = false;
			bool no_change_to_target_angle = false;

			if (arg->drive_position)
			{
				old_sign_from_target_distance = arg->target_distance - arg->end_distance > 0;
				old_sign_from_target_angle = arg->target_angle - arg->end_angle > 0;
				no_change_to_target_distance = arg->target_distance == arg->end_distance;
				no_change_to_target_angle = arg->target_angle == arg->end_angle;
			}

			//increment target angle and XY position
			//if speed or turn rate output is saturated or motors are stalled, stop incrementing (avoid excessive overshoot that PID cannot correct)
			if (fabs(arg->speed_error * arg->speed_pid->getKp() + arg->speed_pid->getIntegral() * arg->speed_pid->getKi()) < arg->max_dps
				&& fabs(arg->angle_error * arg->turn_rate_pid->getKp() + arg->turn_rate_pid->getIntegral() * arg->turn_rate_pid->getKi()) < arg->max_dps
				&& !arg->motor_left->stalled_unsafe() && !arg->motor_right->stalled_unsafe())
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

					//stretch the accel/decel for turn rate and speed to match each other
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

				//update target speed using updated accel/decel values
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

				//update target turn rate using updated accel/decel values
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

				//increment/decrement target angle and distance with updated target speed/turn rate values
				if (!no_change_to_target_angle)
					arg->target_angle += time_since_last_loop * arg->target_turn_rate_constrained;
				if (!no_change_to_target_distance)
					arg->target_distance += time_since_last_loop * arg->target_speed_constrained;

				// arg->target_position_x += time_since_last_loop * arg->target_speed * cos(arg->target_angle / 180 * M_PI) * (1 - fabs(arg->angle_output));
				// arg->target_position_y += time_since_last_loop * arg->target_speed * sin(arg->target_angle / 180 * M_PI) * (1 - fabs(arg->angle_output));
			}

			if (arg->drive_position)
			{
				//compare new sign of error between target and end distance/angle
				//change in sign means end point has been exceeded (so it should be capped)
				bool new_sign_from_target_distance = arg->target_distance - arg->end_distance > 0;
				bool new_sign_from_target_angle = arg->target_angle - arg->end_angle > 0;

				if ((old_sign_from_target_distance != new_sign_from_target_distance) || no_change_to_target_distance)
					arg->target_distance = arg->end_distance;

				if ((old_sign_from_target_angle != new_sign_from_target_angle) || no_change_to_target_angle)
					arg->target_angle = arg->end_angle;

				//in an ideal world, the counter should only increment when both errors are in acceptable range
				//however, our control scheme is only capable of hitting both targets SOME of the time
				//so we count it as complete when the motor is already targeting the angle and distance endpoints, and either error is acceptable
				if (arg->target_angle == arg->end_angle && arg->target_distance == arg->end_distance
					&& (fabs(arg->end_angle - arg->current_angle) <= USER_DRIVE_POS_MIN_ERROR_DEG
						|| fabs(arg->end_distance - arg->current_distance) <= USER_DRIVE_POS_MIN_ERROR_MM))
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
