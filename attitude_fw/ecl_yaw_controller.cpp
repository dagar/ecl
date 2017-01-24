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

#include <stdint.h>
#include <float.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>

ECL_YawController::ECL_YawController() :
	ECL_Controller("yaw"),
	_coordinated_min_speed(100.0f),
	_coordinated_method(0)
{
}

ECL_YawController::~ECL_YawController()
{
}

float ECL_YawController::control_attitude(const struct ECL_ControlData &ctl_data)
{
	switch (_coordinated_method) {
	case COORD_METHOD_OPEN:
		return control_attitude_impl_openloop(ctl_data);

	case COORD_METHOD_CLOSEACC:
		return control_attitude_impl_accclosedloop(ctl_data);

	default:
		static hrt_abstime last_print = 0;

		if (ecl_elapsed_time(&last_print) > 5e6) {
			ECL_WARN("invalid param setting FW_YCO_METHOD");
			last_print = ecl_absolute_time();
		}
	}

	return _rate_setpoint;
}

float ECL_YawController::control_attitude_impl_openloop(const struct ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.speed_body_u) &&
	      PX4_ISFINITE(ctl_data.speed_body_v) &&
	      PX4_ISFINITE(ctl_data.speed_body_w) &&
	      PX4_ISFINITE(ctl_data.roll_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.pitch_rate_setpoint))) {

		return _rate_setpoint;
	}

	/* Calculate desired yaw rate from coordinated turn constraint / (no side forces) */
	_rate_setpoint = 0.0f;

	if (sqrtf(ctl_data.speed_body_u * ctl_data.speed_body_u + ctl_data.speed_body_v * ctl_data.speed_body_v +
		  ctl_data.speed_body_w * ctl_data.speed_body_w) > _coordinated_min_speed) {

		float denumerator = (ctl_data.speed_body_u * cosf(ctl_data.roll) * cosf(ctl_data.pitch) +
				     ctl_data.speed_body_w * sinf(ctl_data.pitch));

		if (fabsf(denumerator) > FLT_EPSILON) {
			_rate_setpoint = (ctl_data.speed_body_w * ctl_data.roll_rate_setpoint +
					  9.81f * sinf(ctl_data.roll) * cosf(ctl_data.pitch) +
					  ctl_data.speed_body_u * ctl_data.pitch_rate_setpoint * sinf(ctl_data.roll)) /
					 denumerator;

		}
	}

	// transform setpoint to body angular rates (jacobian)
	float bodyrate_setpoint = -sinf(ctl_data.roll) * ctl_data.pitch_rate_setpoint
							+ cosf(ctl_data.roll) * cosf(ctl_data.pitch) * _rate_setpoint;

	set_desired_bodyrate(bodyrate_setpoint);

	return _rate_setpoint;
}

float ECL_YawController::control_attitude_impl_accclosedloop(const struct ECL_ControlData &ctl_data)
{
	/* dont set a rate setpoint */
	return 0.0f;
}

float ECL_YawController::control_bodyrate(const struct ECL_ControlData &ctl_data)
{
	switch (_coordinated_method) {
	case COORD_METHOD_OPEN:
	case COORD_METHOD_CLOSEACC:
		return control_bodyrate_impl(ctl_data);

	default:
		static hrt_abstime last_print = 0;

		if (ecl_elapsed_time(&last_print) > 5e6) {
			warnx("invalid param setting FW_YCO_METHOD");
			last_print = ecl_absolute_time();
		}
	}

	return constrain(_last_output, -1.0f, 1.0f);
}

float ECL_YawController::control_bodyrate_impl(const struct ECL_ControlData &ctl_data)
{
	// do not calculate control signal with bad inputs
	if (!(PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.pitch_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate) &&
	      PX4_ISFINITE(ctl_data.pitch_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {

		PX4_WARN("not controlling yaw");
		return constrain(_last_output, -1.0f, 1.0f);
	}

	// input conditioning
	float airspeed = constrain_airspeed(ctl_data.airspeed, ctl_data.airspeed_min, ctl_data.airspeed_max);

	/* Close the acceleration loop if _coordinated_method wants this: change body_rate setpoint */
	if (_coordinated_method == COORD_METHOD_CLOSEACC) {
		//XXX: filtering of acceleration?
		_bodyrate_setpoint -= (ctl_data.acc_body_y / (airspeed * cosf(ctl_data.pitch)));
	}

	// calculate body angular rate error
	_rate_error = _bodyrate_setpoint - ctl_data.yaw_rate;

	update_integrator(ctl_data.lock_integrator);

	// apply PI rate controller and store non-limited output
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.scaler +
		       _rate_error * _k_p * ctl_data.scaler * ctl_data.scaler
		       + _integrator;

	return constrain(_last_output, -1.0f, 1.0f);
}
