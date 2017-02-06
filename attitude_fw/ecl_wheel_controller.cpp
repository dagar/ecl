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
 * @file ecl_wheel_controller.cpp
 * Implementation of a simple PID wheel controller for heading tracking.
 *
 * Authors and acknowledgments in header.
 */

#include "ecl_wheel_controller.h"

float ECL_WheelController::control_attitude(const struct ECL_ControlData &ctl_data)
{
	// do not calculate control signal with bad inputs
	if (!(PX4_ISFINITE(ctl_data.yaw_setpoint) &&
	      PX4_ISFINITE(ctl_data.yaw))) {

		return _rate_setpoint;
	}

	// calculate the error
	float yaw_error = ctl_data.yaw_setpoint - ctl_data.yaw;

	// shortest angle (wrap around)
	yaw_error = fmodf(fmodf(yaw_error + M_PI_F, M_TWOPI_F) + M_TWOPI_F, M_TWOPI_F) - M_PI_F;

	// apply P controller: rate setpoint from current error and time constant
	set_desired_rate(yaw_error / _tc);

	return _rate_setpoint;
}

float ECL_WheelController::control_bodyrate(const struct ECL_ControlData &ctl_data)
{
	// do not calculate control signal with bad inputs
	if (!(PX4_ISFINITE(ctl_data.yaw_rate) &&
	      PX4_ISFINITE(ctl_data.groundspeed))) {

		return constrain(_last_output, -1.0f, 1.0f);
	}

	// calculate body angular rate error
	_rate_error = _rate_setpoint - ctl_data.yaw_rate;

	// Use min airspeed to calculate ground speed scaling region.
	// Don't scale below gspd_scaling_trim
	const float gndspd_scaling_trim = ctl_data.airspeed_min * 0.6f;
	float gndspd_scaler = 1.0f;

	if (ctl_data.groundspeed < gndspd_scaling_trim) {
		gndspd_scaler = gndspd_scaling_trim / gndspd_scaling_trim;

	} else {
		gndspd_scaler = gndspd_scaling_trim / ctl_data.groundspeed;
	}

	const float min_speed = 1.0f;
	update_integrator(ctl_data.lock_integrator && ctl_data.groundspeed > min_speed);

	// apply PI rate controller and store non-limited output
	_last_output = _rate_setpoint * _k_ff * gndspd_scaler +
		       (_rate_error * _k_p + _integrator) * gndspd_scaler * gndspd_scaler;

	return constrain(_last_output, -1.0f, 1.0f);
}

float ECL_WheelController::control_euler_rate(const struct ECL_ControlData &ctl_data)
{
	return 0.0f;
}
