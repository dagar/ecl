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
 * @file ecl_roll_controller.cpp
 * Implementation of a simple orthogonal roll PID controller.
 *
 * Authors and acknowledgments in header.
 */

#include "ecl_roll_controller.h"

float
ECL_RollController::control_attitude(const struct ECL_ControlData &ctl_data)
{
	// do not calculate control signal with bad inputs
	if (!(PX4_ISFINITE(ctl_data.roll_setpoint) &&
	      PX4_ISFINITE(ctl_data.roll))) {

		return _rate_setpoint;
	}

	// calculate the error
	const float roll_error = ctl_data.roll_setpoint - ctl_data.roll;

	// apply P controller: rate setpoint from current error and time constant
	_rate_setpoint = roll_error / _tc;

	return _rate_setpoint;
}

float
ECL_RollController::control_bodyrate(const struct ECL_ControlData &ctl_data)
{
	// do not calculate control signal with bad inputs
	if (!(PX4_ISFINITE(ctl_data.roll_rate) &&
	      PX4_ISFINITE(ctl_data.airspeed_scaler))) {

		return constrain(_last_output, -1.0f, 1.0f);
	}

	// calculate body angular rate error
	_rate_error = _bodyrate_setpoint - ctl_data.roll_rate;

	update_integrator(ctl_data.lock_integrator);

	// apply PI rate controller and store non-limited output
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.airspeed_scaler
		       + (_rate_error * _k_p + _integrator) * ctl_data.airspeed_scaler * ctl_data.airspeed_scaler;

	return constrain(_last_output, -1.0f, 1.0f);
}

float
ECL_RollController::get_desired_bodyrate(const struct ECL_ControlData &ctl_data)
{
	// Transform setpoint to body angular rates (jacobian)
	set_desired_bodyrate(_rate_setpoint - sinf(ctl_data.pitch) * ctl_data.yaw_rate_setpoint);

	return _bodyrate_setpoint;
}
