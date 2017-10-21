/****************************************************************************
 *
 *   Copyright (c) 2017 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tecs.cpp
 *
 * @author Paul Riseborough
 */

#pragma once

#include <stdint.h>

class __EXPORT TECS
{
public:
	TECS() = default;
	~TECS() = default;

	// no copy, assignment, move, move assignment
	TECS(const TECS &) = delete;
	TECS &operator=(const TECS &) = delete;
	TECS(TECS &&) = delete;
	TECS &operator=(TECS &&) = delete;

	/**
	 * Updates the following vehicle kinematic state estimates:
	 * Vertical position, velocity and acceleration.
	 * Speed derivative
	 * Must be called prior to updating tecs control loops
	 * Must be called at 50Hz or greater
	 */
	void update_vehicle_state_estimates(float indicated_airspeed, float eas_to_tas, float ax_body, bool in_air,
					    float altitude, bool vz_valid, float vz, float az);

	void update_height(float altitude, bool vz_valid, float vz, float az);

	/**
	 * Update the airspeed state using a second order complementary filter
	 */
	void update_airspeed(float indicated_airspeed, float EAS2TAS);

	/**
	 * Update the control loop calculations
	 */
	void update_pitch_throttle(float hgt_setpoint, float EAS_setpoint, bool climb_out_mode, float pitch_min_climbout,
				   float throttle_min, float throttle_max, float throttle_cruise, float pitch_limit_min, float pitch_limit_max);

	/**
	 * Handle the altitude reset
	 *
	 * If the estimation system resets the height in one discrete step this
	 * will gracefully even out the reset over time.
	 */
	void handle_alt_step(float delta_alt, float altitude);

	float get_throttle_setpoint(void) { return _last_throttle_setpoint; }
	float get_pitch_setpoint() { return _last_pitch_setpoint; }
	float get_speed_weight() { return _pitch_speed_weight; }

	void reset_state() { _update_timestamp = 0; }

	enum class ECL_TECS_MODE {
		NORMAL = 0,
		UNDERSPEED,
		BAD_DESCENT,
		CLIMBOUT
	};

	void set_time_const(float time_const) { _pitch_time_constant = time_const; }
	void set_time_const_throt(float time_const_throt) { _throttle_time_constant = time_const_throt; }
	void set_integrator_gain(float gain) { _integrator_gain = gain; }

	void set_detect_underspeed_enabled(bool enabled) { _detect_underspeed_enabled = enabled; }
	void set_airspeed_enabled(bool enabled) { _airspeed_enabled = enabled; }

	void set_height_comp_filter_omega(float omega) { _hgt_estimate_freq = omega; }
	void set_heightrate_ff(float heightrate_ff) { _height_setpoint_gain_ff = heightrate_ff; }
	void set_heightrate_p(float heightrate_p) { _height_error_gain = heightrate_p; }

	void set_indicated_airspeed_max(float airspeed) { _indicated_airspeed_max = airspeed; }
	void set_indicated_airspeed_min(float airspeed) { _indicated_airspeed_min = airspeed; }

	void set_max_climb_rate(float climb_rate) { _max_climb_rate = climb_rate; }

	void set_max_sink_rate(float sink_rate) { _max_sink_rate = sink_rate; }
	void set_min_sink_rate(float rate) { _min_sink_rate = rate; }

	void set_pitch_damping(float damping) { _pitch_damping_gain = damping; }
	void set_roll_throttle_compensation(float compensation) { _load_factor_correction = compensation; }

	void set_speed_comp_filter_omega(float omega) { _tas_estimate_freq = omega; }
	void set_speed_weight(float weight) { _pitch_speed_weight = weight; }
	void set_speedrate_p(float speedrate_p) { _speed_error_gain = speedrate_p; }

	void set_throttle_damp(float throttle_damp) { _throttle_damping_gain = throttle_damp; }
	void set_throttle_slewrate(float slewrate) { _throttle_slewrate = slewrate; }

	void set_vertical_accel_limit(float limit) { _vert_accel_limit = limit; }

	// TECS status
	uint64_t timestamp() { return _update_timestamp; }
	ECL_TECS_MODE tecs_mode() { return _tecs_mode; }

	float hgt_setpoint_adj() { return _hgt_setpoint_adj; }
	float vert_pos_state() { return _vert_pos_state; }

	float TAS_setpoint_adj() { return _TAS_setpoint_adj; }
	float tas_state() { return _tas_state; }

	float hgt_rate_setpoint() { return _hgt_rate_setpoint; }
	float vert_vel_state() { return _vert_vel_state; }

	float TAS_rate_setpoint() { return _TAS_rate_setpoint; }
	float speed_derivative() { return _speed_derivative; }

	float STE_error() { return _STE_error; }
	float STE_rate_error() { return _STE_rate_error; }
	float SEB_error() { return _SEB_error; }
	float SEB_rate_error() { return _SEB_rate_error; }

	float throttle_integ_state() { return _throttle_integ_state; }
	float pitch_integ_state() { return _pitch_integ_state; }

private:

	ECL_TECS_MODE _tecs_mode{ECL_TECS_MODE::NORMAL};

	// time steps (non-fixed)
	static constexpr float DT_MIN = 0.001f;		///< minimum allowed value of _dt (sec)
	static constexpr float DT_DEFAULT = 0.02f;	///< default value for _dt (sec)
	static constexpr float DT_MAX = 1.0f;		///< max value of _dt allowed before a filter state reset is performed (sec)

	// timestamps
	uint64_t _update_timestamp{0};			///< last update timestamp

	static constexpr float AIRSPEED_MIN = 3.0f;

	// controller parameters
	float _hgt_estimate_freq{0.0f};					///< cross-over frequency of the height rate complementary filter (rad/sec)
	float _tas_estimate_freq{0.0f};					///< cross-over frequency of the true airspeed complementary filter (rad/sec)
	float _max_climb_rate{2.0f};						///< climb rate produced by max allowed throttle (m/sec)
	float _min_sink_rate{1.0f};						///< sink rate produced by min allowed throttle (m/sec)
	float _max_sink_rate{2.0f};						///< maximum safe sink rate (m/sec)
	float _pitch_time_constant{5.0f};					///< control time constant used by the pitch demand calculation (sec)
	float _throttle_time_constant{8.0f};				///< control time constant used by the throttle demand calculation (sec)
	float _pitch_damping_gain{0.0f};					///< damping gain of the pitch demand calculation (sec)
	float _throttle_damping_gain{0.0f};				///< damping gain of the throttle demand calculation (sec)
	float _integrator_gain{0.0f};						///< integrator gain used by the throttle and pitch demand calculation
	float _vert_accel_limit{0.0f};					///< magnitude of the maximum vertical acceleration allowed (m/sec**2)
	float _load_factor_correction{0.0f};				///< gain from normal load factor increase to total energy rate demand (m**2/sec**3)
	float _pitch_speed_weight{1.0f};					///< speed control weighting used by pitch demand calculation
	float _height_error_gain{0.0f};					///< gain from height error to demanded climb rate (1/sec)
	float _height_setpoint_gain_ff{0.0f};				///< gain from height demand derivative to demanded climb rate
	float _speed_error_gain{0.0f};					///< gain from speed error to demanded speed rate (1/sec)

	// complimentary filter states
	float _vert_accel_state{0.0f};					///< complimentary filter state - height second derivative (m/sec**2)
	float _vert_vel_state{0.0f};						///< complimentary filter state - height rate (m/sec)
	float _vert_pos_state{0.0f};						///< complimentary filter state - height (m)
	float _tas_rate_state{0.0f};						///< complimentary filter state - true airspeed first derivative (m/sec**2)
	float _tas_state{0.0f};							///< complimentary filter state - true airspeed (m/sec)

	// controller states
	float _throttle_integ_state{0.0f};				///< throttle integrator state
	float _pitch_integ_state{0.0f};					///< pitch integrator state (rad)
	float _last_throttle_setpoint{0.0f};				///< throttle demand rate limiter state (1/sec)
	float _last_pitch_setpoint{0.0f};					///< pitch demand rate limiter state (rad/sec)
	float _speed_derivative{0.0f};					///< rate of change of speed along X axis (m/sec**2)

	// speed demand calculations
	float _TAS_setpoint_adj{0.0f};					///< true airspeed demand tracked by the TECS algorithm (m/sec)
	float _TAS_rate_setpoint{0.0f};					///< true airspeed rate demand tracked by the TECS algorithm (m/sec**2)

	float _indicated_airspeed_min{3.0f};				///< equivalent airspeed demand lower limit (m/sec)
	float _indicated_airspeed_max{30.0f};				///< equivalent airspeed demand upper limit (m/sec)
	const float get_true_airspeed_max() const { return _indicated_airspeed_max * _EAS2TAS; }
	const float get_true_airspeed_min() const { return _indicated_airspeed_min * _EAS2TAS; }

	// height demand calculations
	float _hgt_setpoint_adj{0.0f};					///< demanded height used by the control loops after all filtering has been applied (m)
	float _hgt_setpoint_in_prev{0.0f};				///< previous value of _hgt_setpoint after noise filtering (m)
	float _hgt_rate_setpoint{0.0f};					///< demanded climb rate tracked by the TECS algorithm

	// vehicle physical limits
	float _pitch_setpoint_unc{0.0f};					///< pitch demand before limiting (rad)
	float _throttle_slewrate{0.0f};					///< throttle demand slew rate limit (1/sec)

	// specific energy quantities
	float _STE_rate{0.0f};							///< specific total energy rate estimate (m**2/sec**3)

	// specific energy error quantities
	float _STE_setpoint{0.0f};							///< specific total energy setpoint (m**2/sec**2)
	float _STE_error{0.0f};							///< specific total energy error (m**2/sec**2)
	float _STE_rate_setpoint{0.0f};						///< specific total energy rate error (m**2/sec**3)
	float _STE_rate_error{0.0f};						///< specific total energy rate error (m**2/sec**3)

	float _SEB_error{0.0f};							///< specific energy balance error (m**2/sec**2)
	float _SEB_rate_error{0.0f};						///< specific energy balance rate error (m**2/sec**3)
	float _SEB_rate_setpoint{0.0f};						///< specific energy balance rate setpoint (m**2/sec**3)

	// controller mode logic
	bool _underspeed_detected{false};					///< true when an underspeed condition has been detected
	bool _detect_underspeed_enabled{true};			///< true when underspeed detection is enabled
	bool _airspeed_enabled{false};						///< true when airspeed use has been enabled

	float _EAS2TAS{1.0f};

	/**
	 * Update the desired setpoints
	 */
	void _update_airspeed_setpoint(const float airspeed_setpoint);
	void _update_height_setpoint(float height_setpoint, bool climbout_mode);

	/**
	 * Detect if the system is not capable of maintaining airspeed
	 */
	bool underspeed(const float throttle_setpoint_max);
	bool uncommanded_descent(const float throttle_setpoint_max);

	/**
	 * Update specific energy
	 */
	void _update_energy_estimates(bool climbout_mode);

	void _update_throttle_setpoint(const float dt, const float throttle_cruise, const float throttle_min,
				       const float throttle_max,
				       const bool climbout_mode);
	void _update_pitch_setpoint(const float dt, const float pitch_min, const float pitch_max, bool climbout_mode);

};
