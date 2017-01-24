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

#include <stdint.h>
#include <float.h>

ECL_WheelController::ECL_WheelController() :
	ECL_Controller("wheel")
{
}

ECL_WheelController::~ECL_WheelController()
{
}

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
	yaw_error = (float)fmod((float)fmod((yaw_error + M_PI_F), M_TWOPI_F) + M_TWOPI_F, M_TWOPI_F) - M_PI_F;

	// apply P controller: rate setpoint from current error and time constant
	_rate_setpoint =  yaw_error / _tc;

	set_desired_bodyrate(_rate_setpoint);

	return _rate_setpoint;
}

float ECL_WheelController::control_bodyrate(const struct ECL_ControlData &ctl_data)
{
	// do not calculate control signal with bad inputs
	if (!(PX4_ISFINITE(ctl_data.yaw_rate) &&
	      PX4_ISFINITE(ctl_data.groundspeed) &&
	      PX4_ISFINITE(ctl_data.groundspeed_scaler))) {

		return constrain(_last_output, -1.0f, 1.0f);
	}

	// input conditioning
	float min_speed = 1.0f;

	// calculate body angular rate error
	_rate_error = _rate_setpoint - ctl_data.yaw_rate;

	if (ctl_data.groundspeed > min_speed) {
		update_integrator(ctl_data.lock_integrator);
	}

	// apply PI rate controller and store non-limited output
	_last_output = _rate_setpoint * _k_ff * ctl_data.groundspeed_scaler +
		       _rate_error * _k_p * ctl_data.groundspeed_scaler * ctl_data.groundspeed_scaler +
		       _integrator;

	return constrain(_last_output, -1.0f, 1.0f);
}
