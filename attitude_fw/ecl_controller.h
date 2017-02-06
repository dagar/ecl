/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * @file ecl_controller.h
 * Definition of base class for other controllers
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 * Acknowledgments:
 *
 *   The control design is based on a design
 *   by Paul Riseborough and Andrew Tridgell, 2013,
 *   which in turn is based on initial work of
 *   Jonathan Challinger, 2012.
 */

#ifndef ECL_CONTROLLER_H
#define ECL_CONTROLLER_H

#include <ecl/ecl.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>

using math::constrain;
using math::radians;
using math::max;
using math::min;

struct ECL_ControlData {
	float roll;
	float pitch;
	float yaw;
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
	float roll_setpoint;
	float pitch_setpoint;
	float yaw_setpoint;
	float roll_rate_setpoint;
	float pitch_rate_setpoint;
	float yaw_rate_setpoint;
	float airspeed_min;
	float airspeed_max;
	float airspeed;
	float airspeed_scaler;
	float acc_body_y;
	float groundspeed;
	bool lock_integrator;
};

class __EXPORT ECL_Controller
{
public:
	ECL_Controller() = default;
	virtual ~ECL_Controller() = default;

	virtual float control_attitude(const struct ECL_ControlData &ctl_data) = 0;
	virtual float control_euler_rate(const struct ECL_ControlData &ctl_data) = 0;
	virtual float control_bodyrate(const struct ECL_ControlData &ctl_data) = 0;

	void set_time_constant(float time_constant);
	void set_k_p(float k_p);
	void set_k_i(float k_i);
	void set_k_ff(float k_ff);
	void set_integrator_max(float max);

	void set_max_rate(float max_rate);
	void set_max_rate_pos(float max_rate_pos);
	void set_max_rate_neg(float max_rate_neg);

	void set_desired_bodyrate(float body_rate);
	void set_desired_rate(float rate);

	float get_rate_error();
	float get_desired_rate();
	float get_desired_bodyrate();

	void reset_integrator();

protected:
	hrt_abstime _last_run{0};
	float _tc{0.5f};
	float _k_p{0.0f};
	float _k_i{0.0f};
	float _k_ff{0.0f};
	float _integrator_max{0.0f};
	float _max_rate_pos{0.0f};
	float _max_rate_neg{0.0f};
	float _last_output{0.0f};
	float _integrator{0.0f};
	float _rate_error{0.0f};
	float _rate_setpoint{0.0f};
	float _bodyrate_setpoint{0.0f};

	float constrain_airspeed(float airspeed, float minspeed, float maxspeed);

	void update_integrator(bool lock);
};

#endif // ECL_CONTROLLER_H
