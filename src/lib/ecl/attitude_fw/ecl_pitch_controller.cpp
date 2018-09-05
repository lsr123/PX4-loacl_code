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
 * @file ecl_pitch_controller.cpp
 * Implementation of a simple orthogonal pitch PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_pitch_controller.h"
#include <math.h>
#include <stdint.h>
#include <float.h>
#include <geo/geo.h>
#include <ecl/ecl.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>
#include <uORB/topics/ADRC.h>
#include <uORB/uORB.h>

float ADRC_data[5] = {0.0f};


ECL_PitchController::ECL_PitchController() :
	ECL_Controller("pitch"),
	_max_rate_neg(0.0f),
	_roll_ff(0.0f)
{
}

ECL_PitchController::~ECL_PitchController()
{
}

float ECL_PitchController::control_attitude(const struct ECL_ControlData &ctl_data)
{

	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.pitch_setpoint) &&
	      PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.airspeed))) {
		warnx("not controlling pitch");
		return _rate_setpoint;
	}

	/* Calculate the error */
	float pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch;

	/*  Apply P controller: rate setpoint from current error and time constant */
	_rate_setpoint =  pitch_error / _tc;

	/* limit the rate */
	if (_max_rate > 0.01f && _max_rate_neg > 0.01f) {
		if (_rate_setpoint > 0.0f) {
			_rate_setpoint = (_rate_setpoint > _max_rate) ? _max_rate : _rate_setpoint;

		} else {
			_rate_setpoint = (_rate_setpoint < -_max_rate_neg) ? -_max_rate_neg : _rate_setpoint;
		}

	}

	return _rate_setpoint;
}
float ECL_PitchController::MFAC_control(const struct ECL_ControlData &ctl_data)
{
	static float u_a = 0.0f;  // k  time  control signal
	static float u_b = 0.0f;  //k-1 time control signal
	static float u_c = 0.0f;  //k-2 time control signal
	static float y_a = 2.0f;      //k time out
	static float y_b = -1.0f;    //k-1 time out
	static float y_A = 0.0f;    // temp variable
	static float phy_1 = 2.0f; // PPD init value
              static float phy = 2.0f;    //PPD

              y_b = y_A;
              y_a = ctl_data.pitch;
              y_A = y_a;

              phy = phy + _n*(u_b-u_c)/(_miu+(u_b-u_c)*(u_b-u_c))*(y_a-y_b-phy*(u_b-u_c));
              if (abs(phy)<=_kci || abs(u_b-u_c)<=_kci || phy < 0.0f)
              {
              	phy = phy_1;
              }
              u_a = u_b + _rio*phy/(_lamda + abs(phy)*abs(phy))*(ctl_data.pitch_setpoint - y_a);

              u_c = u_b;
              u_b = u_a;

              return math::constrain(u_a, -1.0f, 1.0f);
	
}

float ECL_PitchController::ADRC_control(const struct ECL_ControlData &ctl_data, float P_U)
{
	float u = 0.0f;
	struct ADRC_s ADRC_log;
	

	P_TD(ctl_data);   //得到目标指令的值和微分值   存储到ADRC_data[0]和[1]中
	P_LESO(ctl_data,P_U);  //得到系统输出的值和微分值   存储到ADRC_data[2]和[3]中
	//u = P_ADRC_PD();           //直接进行PD控制
	u = P_ADRC_NLF();            //进行非线性反馈控制
	//printf("%f\n",(double)ADRC_data[0] );
	//printf("%f\n",(double)deltaT );

	memset(&ADRC_log, 0, sizeof(ADRC_log));
	orb_advert_t ADRC_sub = orb_advertise(ORB_ID(ADRC), &ADRC_log);   //公告主题 test_pub为指针
	//ADRC_log.x1 = 0.88;
	ADRC_log.x1 = ADRC_data[0];
	ADRC_log.x2 = ADRC_data[1];
	ADRC_log.z1 = ADRC_data[2];
	ADRC_log.z2 = ADRC_data[3];
	ADRC_log.z3 = ADRC_data[4];
	orb_publish(ORB_ID(ADRC), ADRC_sub, &ADRC_log);      //发布数据
	
	return math::constrain(u, -1.0f, 1.0f);
}
float ECL_PitchController::P_ADRC_PD()
{
	//printf("%f\n",(double)_p);
	//printf("%f\n",(double)_d);
	float u = 0.0f;
	u = _p*(ADRC_data[0]-ADRC_data[2])+_d*(ADRC_data[1]-ADRC_data[3]);
	return u;
	//return math::constrain(u, -1.0f, 1.0f);
}

float ECL_PitchController::P_ADRC_NLF()
{
	float nlf_u = 0.0f;    //非线性反馈的输出控制量
	//nlf_u = (_adrc_lamda * (ADRC_data[0] - ADRC_data[2]) + _adrc_alpha * P_fal(ADRC_data[1] - ADRC_data[3]) - ADRC_data[4]) / _adrc_b0;
	nlf_u = (_adrc_lamda * (ADRC_data[0] - ADRC_data[2]) + _adrc_alpha * P_fal(ADRC_data[1] - ADRC_data[3]) - ADRC_data[4]/100 ) / _adrc_b0;
	return nlf_u;
}

void ECL_PitchController::P_LESO(const struct ECL_ControlData &ctl_data,float P_U)
{
	//printf("aaaaaaa");
	static float z1 = 0.0f;
	static float z2 = 0.0f;
	static float z3 = 0.0f;
	float L1 = 0.0f;
	float L2 = 0.0f;
	float L3 = 0.0f;
	float e = 0.0f;

	L1 = _bd*3;
	L2 = _bd*_bd*3;
	L3 = _bd*_bd*_bd*1;

	e = ctl_data.pitch - z1;
	z1 = z1+_timestep*(z2+L1*e);
	z2 = z2+_timestep*(z3+L2*e+P_U);
	z3 = z3+_timestep*(L3*e);
	ADRC_data[2] = z1;
	ADRC_data[3] = z2;
	ADRC_data[4] = z3;
}
void ECL_PitchController::P_TD(const struct ECL_ControlData &ctl_data )
{
	static float x1 = 0.0f;
	static float x2 = 0.0f;

	float fh = 0.0f;
	fh = P_fhan(x1-ctl_data.pitch_setpoint, x2, _r, _s*_timestep);
	x1 = x1+_timestep*x2;
	x2 = x2+_timestep*fh;
	ADRC_data[0] = x1;
	ADRC_data[1] = x2;
}
float ECL_PitchController::P_fhan(float x1,float x2,float r,float h)
{
	float d = 0.0f;
	float d0 = 0.0f;
	float y = 0.0f;
	float a0 = 0.0f;
	float a = 0.0f;
	float u = 0.0f;

	d = r*h;
	d0 = h*d;
	y = x1+h*x2;
	a0 = sqrt(d*d+8*r*abs(y));
	if(abs(y)>d0)
		a = x2+(a0-d)/2*sign(y);
	if(abs(y)<=d0)
		a = x2+y/h;
	if(abs(a)>d)
		u = -r*sign(a);
	if(abs(a)<=d)
		u = -r*a/d;

	return u;
}

float ECL_PitchController::P_fal(float e)
{	//默认fal函数第二个和第三个参数分别是 0.5和0 在函数中没有设计相应的接口
	float out = 0.0f;
	float temp = 0.0f;
	int a;

	temp = abs(e);
	a =sign(e);
	out = sqrt(temp) * a;
	return out;
}

float ECL_PitchController::control_bodyrate(const struct ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.pitch_rate) &&
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
	float dt = (float)dt_micros * 1e-6f;   //s  miao

	/* lock integral for long intervals */
	bool lock_integrator = ctl_data.lock_integrator;

	if (dt_micros > 500000) {
		lock_integrator = true;
	}

	/* Transform setpoint to body angular rates (jacobian) */
	_bodyrate_setpoint = cosf(ctl_data.roll) * _rate_setpoint +
			     cosf(ctl_data.pitch) * sinf(ctl_data.roll) * ctl_data.yaw_rate_setpoint;

	/* apply turning offset to desired bodyrate setpoint*/
	/* flying inverted (wings upside down)*/
	bool inverted = false;
	float constrained_roll;
	/* roll is used as feedforward term and inverted flight needs to be considered */
	if (fabsf(ctl_data.roll) < math::radians(90.0f)) {
		/* not inverted, but numerically still potentially close to infinity */
		constrained_roll = math::constrain(ctl_data.roll, -fabsf(ctl_data.roll_setpoint), fabsf(ctl_data.roll_setpoint));

	} else {
		/* inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity */
		inverted = true;
		/* note: the ranges are extended by 10 deg here to avoid numeric resolution effects */
		if (ctl_data.roll > 0.0f) {
			/* right hemisphere */
			constrained_roll = math::constrain(ctl_data.roll, math::radians(100.0f), math::radians(180.0f));

		} else {
			/* left hemisphere */
			constrained_roll = math::constrain(ctl_data.roll, math::radians(-100.0f), math::radians(-180.0f));
		}
	}

	/* input conditioning */
	float airspeed = constrain_airspeed(ctl_data.airspeed, ctl_data.airspeed_min, ctl_data.airspeed_max);

	/* Calculate desired body fixed y-axis angular rate needed to compensate for roll angle.
	   For reference see Automatic Control of Aircraft and Missiles by John H. Blakelock, pg. 175
	   Availible on google books 8/11/2015: 
	   https://books.google.com/books?id=ubcczZUDCsMC&pg=PA175#v=onepage&q&f=false*/
	float body_fixed_turn_offset = (fabsf((CONSTANTS_ONE_G / airspeed) *
				  		tanf(constrained_roll) * sinf(constrained_roll)));

	if (inverted) {
		body_fixed_turn_offset = -body_fixed_turn_offset;
	}

	/* Finally add the turn offset to your bodyrate setpoint*/
	_bodyrate_setpoint += body_fixed_turn_offset;


	_rate_error = _bodyrate_setpoint - ctl_data.pitch_rate;

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

	/* Apply PI rate controller and store non-limited output */
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.scaler +
		       _rate_error * _k_p * ctl_data.scaler * ctl_data.scaler
		       + integrator_constrained;  //scaler is proportional to 1/airspeed
//	warnx("pitch: _integrator: %.4f, _integrator_max: %.4f, airspeed %.4f, _k_i %.4f, _k_p: %.4f", (double)_integrator, (double)_integrator_max, (double)airspeed, (double)_k_i, (double)_k_p);
//	warnx("roll: _last_output %.4f", (double)_last_output);
	return math::constrain(_last_output, -1.0f, 1.0f);
}

float ECL_PitchController::sign(float x)
{
	float a = 0.0f;
	if(abs(x) < 0.00001)
		a  = 0.0f;
	else if(x>0)
		a = 1.0f;
	else if(x<0)
		a = -1.0f;
	
	return a;

}
