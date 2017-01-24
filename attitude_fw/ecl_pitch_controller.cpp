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
 * @file ecl_pitch_controller.cpp
 * Implementation of a simple orthogonal pitch PID controller.
 *
 * Authors and acknowledgments in header.
 */

#include "ecl_pitch_controller.h"

#include <stdint.h>
#include <float.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>

ECL_PitchController::ECL_PitchController() :
	ECL_Controller("pitch")
{
}

ECL_PitchController::~ECL_PitchController()
{
}

float ECL_PitchController::control_attitude(const struct ECL_ControlData &ctl_data)
{
	// do not calculate control signal with bad inputs
	if (!(PX4_ISFINITE(ctl_data.pitch_setpoint) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.yaw_rate_setpoint))) {

		ECL_WARN("not controlling pitch");
		return _rate_setpoint;
	}

	// calculate the error
	float pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch;

	// apply P controller: rate setpoint from current error and time constant
	_rate_setpoint = pitch_error / _tc;

	// transform setpoint to body angular rates (jacobian)
	float bodyrate_setpoint = cosf(ctl_data.roll) * _rate_setpoint +
			     cosf(ctl_data.pitch) * sinf(ctl_data.roll) * ctl_data.yaw_rate_setpoint;

	set_desired_bodyrate(bodyrate_setpoint);

	return _rate_setpoint;
}

float ECL_PitchController::control_bodyrate(const struct ECL_ControlData &ctl_data)
{
	// do not calculate control signal with bad inputs
	if (!(PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.pitch_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {

		return constrain(_last_output, -1.0f, 1.0f);
	}

	/* apply turning offset to desired bodyrate setpoint*/
	/* flying inverted (wings upside down)*/
	bool inverted = false;
	float constrained_roll = 0.0f;

	/* roll is used as feedforward term and inverted flight needs to be considered */
	if (fabsf(ctl_data.roll) < radians(90.0f)) {
		/* not inverted, but numerically still potentially close to infinity */
		constrained_roll = constrain(ctl_data.roll, -fabsf(ctl_data.roll_setpoint), fabsf(ctl_data.roll_setpoint));

	} else {
		/* inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity */
		inverted = true;

		/* note: the ranges are extended by 10 deg here to avoid numeric resolution effects */
		if (ctl_data.roll > 0.0f) {
			/* right hemisphere */
			constrained_roll = constrain(ctl_data.roll, radians(100.0f), radians(180.0f));

		} else {
			/* left hemisphere */
			constrained_roll = constrain(ctl_data.roll, radians(-100.0f), radians(-180.0f));
		}
	}

	/* input conditioning */
	float airspeed = constrain_airspeed(ctl_data.airspeed, ctl_data.airspeed_min, ctl_data.airspeed_max);

	/* Calculate desired body fixed y-axis angular rate needed to compensate for roll angle.
	   For reference see Automatic Control of Aircraft and Missiles by John H. Blakelock, pg. 175
	   Availible on google books 8/11/2015:
	   https://books.google.com/books?id=ubcczZUDCsMC&pg=PA175#v=onepage&q&f=false */
	float body_fixed_turn_offset = (fabsf((CONSTANTS_ONE_G / airspeed) * tanf(constrained_roll) * sinf(constrained_roll)));

	if (inverted) {
		body_fixed_turn_offset = -body_fixed_turn_offset;
	}

	/* Finally add the turn offset to your bodyrate setpoint */
	_bodyrate_setpoint += body_fixed_turn_offset;

	// calculate body angular rate error
	_rate_error = _bodyrate_setpoint - ctl_data.pitch_rate;

	update_integrator(ctl_data.lock_integrator);

	// apply PI rate controller and store non-limited output
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.scaler +
		       _rate_error * _k_p * ctl_data.scaler * ctl_data.scaler
		       + _integrator;

	return constrain(_last_output, -1.0f, 1.0f);
}
