/****************************************************************************
 *
 *   Copyright (c) 2013-2016 Estimation and Control Library (ECL). All rights reserved.
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
 * Authors and acknowledgements in header.
 */

#include <ecl/ecl.h>
#include "ecl_roll_controller.h"
#include <stdint.h>
#include <float.h>
#include <geo/geo.h>
#include <ecl/ecl.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>

ECL_RollController::ECL_RollController() :
	ECL_Controller("roll")
{
}

ECL_RollController::~ECL_RollController()
{
}
float ECL_RollController::MFAC_control(const struct ECL_ControlData &ctl_data)
{
	//static float u_a = 0.0f;  // k  time  controlsignal
	//static float u_b = 0.0f;  //k-1 time controlsignal
	//static float u_c = 0.0f;  //k-2 time controlsignal
	//static float y_a = 2.0f;      //k time out
	//static float y_b = -1.0f;    //k-1 time out
	//static float y_A = 0.0f;    // temp variable
              static float phy = 2.0f;    //PPD


           //   y_b = y_A;
             // y_a = ctl_data.pitch;
              //y_A = y_a;

              phy = phy + _lamda;



	return phy;
}
float ECL_RollController::ADRC_control(const struct ECL_ControlData &ctl_data,float R_U)
{
	return 0;
}

float ECL_RollController::control_attitude(const struct ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll_setpoint) && PX4_ISFINITE(ctl_data.roll))) {
		return _rate_setpoint;
	}

	/* Calculate error */
	float roll_error = ctl_data.roll_setpoint - ctl_data.roll;

	/* Apply P controller */
	_rate_setpoint = roll_error / _tc;

	/* limit the rate */ //XXX: move to body angluar rates

	if (_max_rate > 0.01f) {
		_rate_setpoint = (_rate_setpoint > _max_rate) ? _max_rate : _rate_setpoint;
		_rate_setpoint = (_rate_setpoint < -_max_rate) ? -_max_rate : _rate_setpoint;
	}

	return _rate_setpoint;
}

float ECL_RollController::control_bodyrate(const struct ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.roll_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {
		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* get the usual dt estimate */
	uint64_t dt_micros = ecl_elapsed_time(&_last_run);
	_last_run = ecl_absolute_time();
	float dt = (float)dt_micros * 1e-6f;

	/* lock integral for long intervals */
	bool lock_integrator = ctl_data.lock_integrator;

	if (dt_micros > 500000) {
		lock_integrator = true;
	}

	/* Transform setpoint to body angular rates (jacobian) */
	_bodyrate_setpoint = _rate_setpoint - sinf(ctl_data.pitch) * ctl_data.yaw_rate_setpoint;

	/* Calculate body angular rate error */
	_rate_error = _bodyrate_setpoint - ctl_data.roll_rate; //body angular rate error

	if (!lock_integrator && _k_i > 0.0f) {

		float id = _rate_error * dt * ctl_data.scaler;

		/*
		* anti-windup: do not allow integrator to increase if actuator is at limit
		*/
		if (_last_output < -1.0f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		_integrator += id * _k_i;
	}

	/* integrator limit */
	//xxx: until start detection is available: integral part in control signal is limited here
	float integrator_constrained = math::constrain(_integrator, -_integrator_max, _integrator_max);
	//warnx("roll: _integrator: %.4f, _integrator_max: %.4f", (double)_integrator, (double)_integrator_max);

	/* Apply PI rate controller and store non-limited output */
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.scaler +
		       _rate_error * _k_p * ctl_data.scaler * ctl_data.scaler
		       + integrator_constrained;  //scaler is proportional to 1/airspeed

	return math::constrain(_last_output, -1.0f, 1.0f);
}

