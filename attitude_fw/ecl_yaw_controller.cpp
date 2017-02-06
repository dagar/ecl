/****************************************************************************
 *
 *   Copyright (c) 2013-2017 Estimation and Control Library (ECL). All rights reserved.
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
 * @file ecl_yaw_controller.cpp
 * Implementation of a simple orthogonal coordinated turn yaw PID controller.
 *
 * Authors and acknowledgments in header.
 */

#include "ecl_yaw_controller.h"

float
ECL_YawController::control_attitude(const struct ECL_ControlData &ctl_data)
{
	if (_coordinated_method == COORD_METHOD_OPEN) {
		// do not calculate control signal with bad inputs
		if (!(PX4_ISFINITE(ctl_data.roll) &&
		      PX4_ISFINITE(ctl_data.pitch) &&
		      PX4_ISFINITE(ctl_data.roll_setpoint) &&
		      PX4_ISFINITE(ctl_data.airspeed) &&
		      PX4_ISFINITE(ctl_data.airspeed_min) &&
		      PX4_ISFINITE(ctl_data.airspeed_max))) {

			return _rate_setpoint;
		}

		float constrained_roll = 0.0f;
		bool inverted = false;

		// roll is used as feedforward term and inverted flight needs to be considered
		if (fabsf(ctl_data.roll) < radians(90.0f)) {
			// not inverted, but numerically still potentially close to infinity
			constrained_roll = constrain(ctl_data.roll, radians(-60.0f), radians(60.0f));

		} else {
			inverted = true;

			// inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity
			// note: the ranges are extended by 40 deg here to avoid numeric resolution effects
			if (ctl_data.roll > 0.0f) {
				// right hemisphere
				constrained_roll = constrain(ctl_data.roll, radians(130.0f), radians(180.0f));

			} else {
				// left hemisphere
				constrained_roll = constrain(ctl_data.roll, radians(-180.0f), radians(-130.0f));
			}
		}

		constrained_roll = constrain(constrained_roll, -fabsf(ctl_data.roll_setpoint), fabsf(ctl_data.roll_setpoint));

		const float airspeed = constrain_airspeed(ctl_data.airspeed, ctl_data.airspeed_min, ctl_data.airspeed_max);

		if (!inverted) {
			// Calculate desired yaw rate from coordinated turn constraint / (no side forces)
			_rate_setpoint = tanf(constrained_roll) * cosf(ctl_data.pitch) * CONSTANTS_ONE_G / airspeed;
		}

	} else if (_coordinated_method == COORD_METHOD_CLOSEACC) {
		// rate setpoint from body y acceleration
		_rate_setpoint = 0.0f;

	} else {

		_rate_setpoint = 0.0f;
	}

	return _rate_setpoint;
}

float
ECL_YawController::control_bodyrate(const struct ECL_ControlData &ctl_data)
{
	// do not calculate control signal with bad inputs
	if (!(PX4_ISFINITE(ctl_data.yaw_rate) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.acc_body_y) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.airspeed_scaler))) {

		return constrain(_last_output, -1.0f, 1.0f);
	}

	// Close the acceleration loop if _coordinated_method wants this: change body_rate setpoint */
	if (_coordinated_method == COORD_METHOD_CLOSEACC) {
		//float airspeed = constrain_airspeed(ctl_data.airspeed, ctl_data.airspeed_min, ctl_data.airspeed_max);

		// XXX lateral acceleration needs to go into integrator with a gain
		//_bodyrate_setpoint -= (ctl_data.acc_body_y / (airspeed * cosf(ctl_data.pitch)));
	}

	// calculate body angular rate error
	_rate_error = _bodyrate_setpoint - ctl_data.yaw_rate;

	update_integrator(ctl_data.lock_integrator);

	// apply PI rate controller and store non-limited output
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.airspeed_scaler
		       + (_rate_error * _k_p + _integrator) * ctl_data.airspeed_scaler * ctl_data.airspeed_scaler;

	return constrain(_last_output, -1.0f, 1.0f);
}

float
ECL_YawController::get_desired_bodyrate(const struct ECL_ControlData &ctl_data)
{
	// Transform setpoint to body angular rates (jacobian)
	set_desired_bodyrate(-sinf(ctl_data.roll) * _rate_setpoint
			     + cosf(ctl_data.roll) * cosf(ctl_data.pitch) * _rate_setpoint);

	return _bodyrate_setpoint;
}
