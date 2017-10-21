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

#include "tecs.hpp"

#include <ecl/ecl.h>
#include <geo/geo.h>

using math::constrain;
using math::max;
using math::min;

/**
 * @file tecs.cpp
 *
 * @author Paul Riseborough
 */

/*
 * This function implements a complementary filter to estimate the climb rate when
 * inertial nav data is not available. It also calculates a true airspeed derivative
 * which is used by the airspeed complimentary filter.
 */
void TECS::update_vehicle_state_estimates(float indicated_airspeed, float eas_to_tas,
		float ax_body, bool in_air)
{
	// calculate the time lapsed since the last update
	const uint64_t now = ecl_absolute_time();
	float dt = max((now - _update_timestamp) * 1.0e-6f, 0.0f);

	if (_update_timestamp == 0 || dt < DT_MIN || dt > DT_MAX || !in_air) {

		dt = DT_DEFAULT;

		// On first time through or when not using TECS of if there has been a large time slip,
		// states must be reset to allow filters to a clean start
		_vert_accel_state = 0.0f;

		if (vz_valid) {
			_vert_vel_state = -vz;

		} else {
			_vert_vel_state = 0.0f;
		}

		_vert_pos_state = altitude;
		_tas_rate_state = 0.0f;
		_tas_state = indicated_airspeed * eas_to_tas;

		_throttle_integ_state = 0.0f;
		_pitch_integ_state = 0.0f;
		_last_throttle_setpoint = throttle_cruise;
		_last_pitch_setpoint = constrain(pitch, pitch_min, pitch_max);
		_pitch_setpoint_unc = _last_pitch_setpoint;
		_hgt_setpoint_adj = altitude;
		_hgt_setpoint_in_prev = altitude;
		_TAS_setpoint_adj = indicated_airspeed * eas_to_tas;

		_STE_rate_error = 0.0f;
	}

	_dt = dt;
	_update_timestamp = now;
}

void TECS::update_height(float altitude, bool vz_valid, float vz, float az)
{
	// Generate the height and climb rate state estimates
	if (vz_valid) {
		// Set the velocity and position state to the the INS data
		_vert_vel_state = -vz;
		_vert_pos_state = altitude;

	} else {
		// Get height acceleration
		const float hgt_ddot_mea = -az;

		// If we have no vertical INS data, estimate the vertical velocity using a complementary filter
		// Perform filter calculation using backwards Euler integration
		// Coefficients selected to place all three filter poles at omega
		// Reference Paper: Optimising the Gains of the Baro-Inertial Vertical Channel
		// Widnall W.S, Sinha P.K, AIAA Journal of Guidance and Control, 78-1307R
		const float omega2 = _hgt_estimate_freq * _hgt_estimate_freq;
		const float hgt_err = altitude - _vert_pos_state;
		const float vert_accel_input = hgt_err * omega2 * _hgt_estimate_freq;
		_vert_accel_state += vert_accel_input * dt;
		const float vert_vel_input = _vert_accel_state + hgt_ddot_mea + hgt_err * omega2 * 3.0f;
		_vert_vel_state = _vert_vel_state + vert_vel_input * dt;
		const float vert_pos_input = _vert_vel_state + hgt_err * _hgt_estimate_freq * 3.0f;

		// If more than 1 second has elapsed since last update then reset the position state
		// to the measured height
		if (dt > 1.0f) {
			_vert_pos_state = altitude;

		} else {
			_vert_pos_state = _vert_pos_state + vert_pos_input * dt;
		}
	}
}

void TECS::update_airspeed(float indicated_airspeed, float EAS2TAS)
{
	// If airspeed measurements are not being used, fix the airspeed estimate to halfway between
	// min and max limits

	float TAS = 0.0f;

	if (!ISFINITE(indicated_airspeed) || !_airspeed_enabled) {
		TAS = 0.5f * (_indicated_airspeed_min + _indicated_airspeed_max) * EAS2TAS;

		// Assuming the vehicle is flying X axis forward, use the X axis measured acceleration
		// compensated for gravity to estimate the rate of change of speed
		float speed_deriv_raw = rotMat(2, 0) * CONSTANTS_ONE_G + accel_body(0);

		// Apply some noise filtering
		_speed_derivative = 0.95f * _speed_derivative + 0.05f * speed_deriv_raw;

	} else {
		TAS = indicated_airspeed * EAS2TAS;

		_speed_derivative = 0.0f;
	}

	float tas_state = _tas_state;
	float tas_rate_state = _tas_rate_state;

	// If first time through or not flying, reset airspeed states
	if (_update_timestamp == 0 || !in_air) {
		tas_state = TAS;
		tas_rate_state = 0.0f;
	}

	// Obtain a smoothed airspeed estimate using a second order complementary filter

	// Update TAS rate state
	const float tas_error = TAS - tas_state;
	float tas_rate_state_input = tas_error * _tas_estimate_freq * _tas_estimate_freq;

	// limit integrator input to prevent windup
	if (tas_state <= AIRSPEED_MIN) {
		tas_rate_state_input = max(tas_rate_state_input, 0.0f);
	}

	// Update TAS state
	tas_rate_state += tas_rate_state_input * dt;

	const float tas_state_input = tas_rate_state + _speed_derivative + tas_error * _tas_estimate_freq * sqrtf(2.0f);
	tas_state += tas_state_input * dt;

	// Limit the airspeed state to a minimum of 3 m/s
	_tas_state = max(tas_state, AIRSPEED_MIN);
	_tas_rate_state = tas_rate_state;
}

void TECS::_update_airspeed_setpoint(const float airspeed_setpoint)
{
	float airspeed_setpoint_adjusted = constrain(airspeed_setpoint, get_true_airspeed_min(), get_true_airspeed_max());

	// Set the airspeed demand to the minimum value if an underspeed or
	// or a uncontrolled descent condition exists to maximise climb rate
	if (_tecs_mode == ECL_TECS_MODE::BAD_DESCENT || _tecs_mode == ECL_TECS_MODE::UNDERSPEED) {
		airspeed_setpoint_adjusted = get_true_airspeed_min();
	}

	// Apply limits on the demanded rate of change of speed based based on physical performance limits
	// with a 50% margin to allow the total energy controller to correct for errors.
	const float velRateMax = 0.5f * _max_climb_rate * CONSTANTS_ONE_G / _tas_state;
	const float velRateMin = 0.5f * -_min_sink_rate * CONSTANTS_ONE_G / _tas_state;

	// calculate the demanded rate of change of speed proportional to speed error and apply performance limits
	_TAS_rate_setpoint = constrain((airspeed_setpoint_adjusted - _tas_state) * _speed_error_gain, velRateMin, velRateMax);
	_TAS_setpoint_adj = airspeed_setpoint_adjusted;
}

void TECS::_update_height_setpoint(float height_setpoint_desired, bool climbout_mode)
{
	const float hgt_setpoint_adj_prev = _hgt_setpoint_adj;

	// Detect first time through and initialize previous value to demand
	if (ISFINITE(_hgt_setpoint_in_prev) && fabsf(_hgt_setpoint_in_prev) < 0.1f) {
		_hgt_setpoint_in_prev = height_setpoint_desired;
	}

	// Apply a 2 point moving average to demanded height to reduce
	// intersampling noise effects.
	float hgt_setpoint = _hgt_setpoint_in_prev;

	if (ISFINITE(height_setpoint_desired)) {
		hgt_setpoint = 0.5f * (height_setpoint_desired + _hgt_setpoint_in_prev);
	}

	_hgt_setpoint_in_prev = hgt_setpoint;

	// Apply a rate limit to respect vehicle performance limitations
	hgt_setpoint = constrain(hgt_setpoint, hgt_setpoint_adj_prev - _max_sink_rate * _dt,
				 hgt_setpoint_adj_prev + _max_climb_rate * _dt);

	// Apply a first order noise filter
	const float hgt_setpoint_adj = 0.1f * hgt_setpoint + 0.9f * hgt_setpoint_adj_prev;

	// Calculate the demanded climb rate proportional to height error plus a feedforward term to provide
	// tight tracking during steady climb and descent maneuvers.
	float hgt_rate_setpoint = (hgt_setpoint_adj - _vert_pos_state) * _height_error_gain + _height_setpoint_gain_ff *
				  (hgt_setpoint_adj - hgt_setpoint_adj_prev) / _dt;

	// Limit the rate of change of height demand to respect vehicle performance limits
	_hgt_rate_setpoint = constrain(hgt_rate_setpoint, -_max_sink_rate, _max_climb_rate);
	_hgt_setpoint_adj = hgt_setpoint_adj;
}

bool TECS::underspeed(const float throttle_setpoint_max)
{
	if (!_detect_underspeed_enabled) {
		_underspeed_detected = false;
		return false;
	}

	const bool airspeed_low = (_tas_state < get_true_airspeed_min() * 0.9f);
	const bool throttle_high = (_last_throttle_setpoint >= throttle_setpoint_max * 0.95f);
	const bool alt_low = (_vert_pos_state < _hgt_setpoint_adj);

	if ((airspeed_low && throttle_high) || (alt_low && _underspeed_detected)) {
		_underspeed_detected = true;

	} else {
		_underspeed_detected = false;
	}

	return _underspeed_detected;
}

void TECS::_update_energy_estimates(bool climbout_mode)
{
	// Calculate specific energy demands in units of (m**2/sec**2)
	const float SPE_setpoint = _hgt_setpoint_adj * CONSTANTS_ONE_G; // potential energy
	const float SKE_setpoint = 0.5f * _TAS_setpoint_adj * _TAS_setpoint_adj; // kinetic energy

	// Calculate specific energy rate demands in units of (m**2/sec**3)
	const float SPE_rate_setpoint = _hgt_rate_setpoint * CONSTANTS_ONE_G; // potential energy rate of change
	const float SKE_rate_setpoint = _tas_state * _TAS_rate_setpoint; // kinetic energy rate of change

	// Calculate specific energies in units of (m**2/sec**2)
	const float SPE_estimate = _vert_pos_state * CONSTANTS_ONE_G; // potential energy
	const float SKE_estimate = 0.5f * _tas_state * _tas_state; // kinetic energy

	// Calculate specific energy rates in units of (m**2/sec**3)
	const float SPE_rate = _vert_vel_state * CONSTANTS_ONE_G; // potential energy rate of change
	const float SKE_rate = _tas_state * _speed_derivative; // kinetic energy rate of change

	// Calculate rate of change of total specific energy
	_STE_rate = SPE_rate + SKE_rate;

	const float SPE_ERR = SPE_setpoint - SPE_estimate;
	const float SPE_rate_error = SPE_rate - SPE_rate_setpoint;

	const float SKE_ERR = SKE_setpoint - SKE_estimate;
	const float SKE_rate_error = SKE_rate - SKE_rate_setpoint;



	// Calculate total energy error
	_STE_error = SPE_ERR + SKE_ERR;

	// Adjust the demanded total energy rate to compensate for induced drag rise in turns.
	// Assume induced drag scales linearly with normal load factor.
	// The additional normal load factor is given by (1 / cos(bank angle) - 1)
	const float cosPhi = sqrtf((rotMat(0, 1) * rotMat(0, 1)) + (rotMat(1, 1) * rotMat(1, 1)));
	_STE_rate_setpoint += _load_factor_correction * (1.0f / constrain(cosPhi, 0.1f, 1.0f) - 1.0f);

	// Calculate demanded rate of change of total energy, respecting vehicle limits
	_STE_rate_setpoint = constrain((SPE_rate_setpoint + SKE_rate_setpoint), -_min_sink_rate * CONSTANTS_ONE_G,
				       _max_sink_rate * CONSTANTS_ONE_G);

	// Calculate the total energy rate error, applying a first order IIR filter
	// to reduce the effect of accelerometer noise
	_STE_rate_error = 0.2f * (_STE_rate_setpoint - SPE_rate - SKE_rate) + 0.8f * _STE_rate_error;

	/*
	 * The SKE_weighting variable controls how speed and height control are prioritized by the pitch demand calculation.
	 * A weighting of 1 gives equal speed and height priority
	 * A weighting of 0 gives 100% priority to height control and must be used when no airspeed measurement is available.
	 * A weighting of 2 provides 100% priority to speed control and is used when:
	 * a) an underspeed condition is detected.
	 * b) during climbout where a minimum pitch angle has been set to ensure height is gained. If the airspeed
	 * rises above the demanded value, the pitch angle demand is increased by the TECS controller to prevent the vehicle overspeeding.
	 * The weighting can be adjusted between 0 and 2 depending on speed and height accuracy requirements.
	*/

	// Calculate the weighting applied to control of specific kinetic energy error
	float SKE_weighting = constrain(_pitch_speed_weight, 0.0f, 2.0f);

	if ((_underspeed_detected || climbout_mode) && _airspeed_enabled) {
		SKE_weighting = 2.0f;

	} else if (!_airspeed_enabled) {
		SKE_weighting = 0.0f;
	}

	// Calculate the weighting applied to control of specific potential energy error
	const float SPE_weighting = 2.0f - SKE_weighting;

	// Calculate the specific energy balance demand which specifies how the available total
	// energy should be allocated to speed (kinetic energy) and height (potential energy)
	_SEB_error = SPE_weighting * SPE_ERR - SKE_weighting * SKE_ERR;

	// Calculate the specific energy balance rate demand
	_SEB_rate_setpoint = SPE_weighting * SPE_rate_setpoint - SKE_weighting * SKE_rate_setpoint;
	_SEB_rate_error = SPE_weighting * SPE_rate_error - SKE_weighting * SKE_rate_error;
}

void TECS::_update_throttle_setpoint(const float throttle_cruise, const float throttle_min, const float throttle_max,
				     bool climbout_mode)
{
	const float STE_rate_min = -_min_sink_rate * CONSTANTS_ONE_G;
	const float STE_rate_max = _max_sink_rate * CONSTANTS_ONE_G;

	// Calculate a predicted throttle from the demanded rate of change of energy, using the cruise throttle
	// as the starting point. Assume:
	// Specific total energy rate = STE_rate_max is achieved when throttle is set to to throttle_max
	// Specific total energy rate = 0 at cruise throttle
	// Specific total energy rate = _STE_rate_min is achieved when throttle is set to to throttle_min
	float throttle_predicted = 0.0f;

	if (_STE_rate_setpoint >= 0) {
		// throttle is between cruise and maximum
		throttle_predicted = throttle_cruise + _STE_rate_setpoint / STE_rate_max * (throttle_max - throttle_cruise);

	} else {
		// throttle is between cruise and minimum
		throttle_predicted = throttle_cruise + (_STE_rate_setpoint / STE_rate_min) * (throttle_min - throttle_cruise);
	}

	float throttle_setpoint = throttle_predicted;

	// when flying without an airspeed sensor, use the predicted throttle only
	if (_airspeed_enabled) {

		// Calculate gain scaler from specific energy error to throttle
		const float STE_to_throttle = 1.0f / (_throttle_time_constant * (STE_rate_max - STE_rate_max));

		// Add proportional and derivative control feedback to the predicted throttle and constrain to throttle limits
		throttle_setpoint = (_STE_error + _STE_rate_error * _throttle_damping_gain) * STE_to_throttle + throttle_predicted;
		throttle_setpoint = constrain(throttle_setpoint, throttle_min, throttle_max);

		// Rate limit the throttle demand
		if (fabsf(_throttle_slewrate) > 0.01f) {
			const float throttle_slew_increment_limit = _dt * (throttle_max - throttle_min) * _throttle_slewrate;
			const float throttle_slew_min = _last_throttle_setpoint - throttle_slew_increment_limit;
			const float throttle_slew_max = _last_throttle_setpoint + throttle_slew_increment_limit;

			throttle_setpoint = constrain(throttle_setpoint, throttle_slew_min, throttle_slew_max);
		}

		// Calculate throttle integrator state upper and lower limits with allowance for
		// 10% throttle saturation to accommodate noise on the demand
		const float integ_state_max = (throttle_max - throttle_setpoint + 0.1f);
		const float integ_state_min = (throttle_min - throttle_setpoint - 0.1f);

		// Calculate a throttle demand from the integrated total energy error
		// This will be added to the total throttle demand to compensate for steady state errors
		throttle_integ_state = _throttle_integ_state + (_STE_error * _integrator_gain) * _dt * STE_to_throttle;

		if (climbout_mode) {
			// During climbout, set the integrator to maximum throttle to prevent transient throttle drop
			// at end of climbout when we transition to closed loop throttle control
			_throttle_integ_state = integ_state_max;

		} else {
			// Respect integrator limits during closed loop operation.
			_throttle_integ_state = constrain(throttle_integ_state, integ_state_min, integ_state_max);
		}

		// Add the integrator feedback during closed loop operation with an airspeed sensor
		throttle_setpoint += _throttle_integ_state;
	}

	_last_throttle_setpoint = constrain(throttle_setpoint, throttle_min, throttle_max);
}

bool TECS::uncommanded_descent(const float throttle_setpoint_max)
{
	/*
	 * This function detects a condition that can occur when the demanded airspeed is greater than the
	 * aircraft can achieve in level flight. When this occurs, the vehicle will continue to reduce height
	 * while attempting to maintain speed.
	*/

	// If total energy is very low and reducing, throttle is high, and we are not in an underspeed condition, then enter uncommanded descent recovery mode
	const bool enter_mode = !_underspeed_detected && (_STE_error > 200.0f)
				&& (_STE_rate < 0.0f) && (_last_throttle_setpoint >= throttle_setpoint_max * 0.9f);

	// If we enter an underspeed condition or recover the required total energy, then exit uncommanded descent recovery mode
	const bool exit_mode = (_underspeed_detected || (_STE_error < 0.0f));

	if (enter_mode) {
		return true;

	} else if (exit_mode) {
		return false;
	}

	return (_tecs_mode == ECL_TECS_MODE::BAD_DESCENT);
}

void TECS::_update_pitch_setpoint(const float dt, const float pitch_min, const float pitch_max, bool climbout_mode)
{
	// Calculate derivative from change in climb angle to rate of change of specific energy balance
	const float climb_angle_to_SEB_rate = _tas_state * _pitch_time_constant * CONSTANTS_ONE_G;

	// Calculate pitch integrator input term
	float pitch_integ_input = _SEB_error * _integrator_gain;

	// Prevent the integrator changing in a direction that will increase pitch demand saturation
	// Decay the integrator at the control loop time constant if the pitch demand from the previous time step is saturated
	if (_pitch_setpoint_unc > pitch_max) {
		pitch_integ_input = min(pitch_integ_input,
					min((pitch_max - _pitch_setpoint_unc) * climb_angle_to_SEB_rate / _pitch_time_constant, 0.0f));

	} else if (_pitch_setpoint_unc < pitch_min) {
		pitch_integ_input = max(pitch_integ_input,
					max((pitch_min - _pitch_setpoint_unc) * climb_angle_to_SEB_rate / _pitch_time_constant, 0.0f));
	}

	// Update the pitch integrator state
	_pitch_integ_state += pitch_integ_input * dt;

	// Calculate a specific energy correction that doesn't include the integrator contribution
	float SEB_correction = _SEB_error + _SEB_rate_error * _pitch_damping_gain + _SEB_rate_setpoint * _pitch_time_constant;

	// During climbout, bias the demanded pitch angle so that a zero speed error produces a pitch angle
	// demand equal to the minimum pitch angle set by the mission plan. This prevents the integrator
	// having to catch up before the nose can be raised to reduce excess speed during climbout.
	if (climbout_mode) {
		SEB_correction += pitch_min * climb_angle_to_SEB_rate;
	}

	// Sum the correction terms and convert to a pitch angle demand. This calculation assumes:
	// a) The climb angle follows pitch angle with a lag that is small enough not to destabilise the control loop.
	// b) The offset between climb angle and pitch angle (angle of attack) is constant, excluding the effect of
	// pitch transients due to control action or turbulence.
	_pitch_setpoint_unc = (SEB_correction + _pitch_integ_state) / climb_angle_to_SEB_rate;

	const float pitch_setpoint = constrain(_pitch_setpoint_unc, pitch_min, pitch_max);

	// Comply with the specified vertical acceleration limit by applying a pitch rate limit
	const float ptchRateIncr = dt * _vert_accel_limit / _tas_state;

	_last_pitch_setpoint = constrain(pitch_setpoint, _last_pitch_setpoint - ptchRateIncr,
					 _last_pitch_setpoint + ptchRateIncr);
}

void TECS::update_pitch_throttle(float hgt_setpoint, float EAS_setpoint, bool climbout_mode,
				 float pitch_min_climbout, float throttle_min, float throttle_max, float throttle_cruise, float pitch_min,
				 float pitch_max)
{
	// Set TECS mode for next frame
	if (underspeed(throttle_max)) {
		_tecs_mode = ECL_TECS_MODE::UNDERSPEED;

		_TAS_setpoint_adj = get_true_airspeed_min();

	} else if (uncommanded_descent(throttle_max)) {
		_tecs_mode = ECL_TECS_MODE::BAD_DESCENT;

		_TAS_setpoint_adj = get_true_airspeed_min();

	} else if (climbout_mode) {
		_tecs_mode = ECL_TECS_MODE::CLIMBOUT;

		// airspeed demand states are set to track the measured airspeed
		_TAS_setpoint_adj = _tas_state;
		_TAS_rate_setpoint = 0.0f;

		// height demand and associated states are set to track the measured height
		_hgt_setpoint_adj = _vert_pos_state;

	} else {
		// This is the default operation mode
		_tecs_mode = ECL_TECS_MODE::NORMAL;
	}

	// Calculate the demanded true airspeed
	_update_airspeed_setpoint(EAS_setpoint);

	// Calculate the demanded height
	_update_height_setpoint(hgt_setpoint, climbout_mode);

	// Calculate the specific energy values required by the control loop
	_update_energy_estimates(rotMat);

	// Calculate the throttle demand

	// Calculate the throttle demand
	if (_tecs_mode == ECL_TECS_MODE::UNDERSPEED) {
		// always use full throttle to recover from an underspeed condition
		_last_throttle_setpoint = 1.0f;

	} else {
		_update_throttle_setpoint(dt, throttle_cruise, throttle_min, throttle_max, climbout_mode);
	}

	// Calculate the pitch demand
	_update_pitch_setpoint(dt, pitch_min, pitch_max, climbout_mode);
}

void TECS::handle_alt_step(float delta_alt, float altitude)
{
	// add height reset delta to all variables involved
	// in filtering the demanded height
	_hgt_setpoint_in_prev += delta_alt;

	// reset height states
	_vert_pos_state = altitude;
	_vert_accel_state = 0.0f;
	_vert_vel_state = 0.0f;
}
