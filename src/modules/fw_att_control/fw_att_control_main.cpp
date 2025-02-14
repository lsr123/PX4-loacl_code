/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file fw_att_control_main.c
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler 	<thomasgubler@gmail.com>
 * @author Roman Bapst		<bapstr@ethz.ch>
 *
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>
#include <ecl/attitude_fw/ecl_pitch_controller.h>
#include <ecl/attitude_fw/ecl_roll_controller.h>
#include <ecl/attitude_fw/ecl_wheel_controller.h>
#include <ecl/attitude_fw/ecl_yaw_controller.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/fw_virtual_attitude_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/ADRC.h>
#include <uORB/uORB.h>



#include <sys/ipc.h>



#include <px4_config.h>
#include <px4_posix.h>

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
//#include <nuttx/config.h>
//#include <nuttx/ascii.h>
//#include <nuttx/compiler.h>


/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[]);

class FixedwingAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingAttitudeControl();

	/**
	 * Destructor, also kills the main task.
	 */
	~FixedwingAttitudeControl();

	/**
	 * Start the main task.
	 *
	 * @return	PX4_OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool		_task_should_exit;		/**< if true, attitude control task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle */

	int		_accel_sub;			/**< accelerometer subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_battery_status_sub;		/**< battery status subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_global_pos_sub;		/**< global position subscription */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_params_sub;			/**< notification of parameter updates */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */

	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_2_pub;		/**< actuator control group 1 setpoint (Airframe) */

	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure
	orb_id_t _attitude_setpoint_id;

	struct accel_report				_accel;			/**< body frame accelerations */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;	/**< actuator control inputs */
	struct battery_status_s				_battery_status;	/**< battery status */
	struct control_state_s				_ctrl_state;	/**< control state */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_control_mode_s			_vcontrol_mode;		/**< vehicle control mode */
	struct vehicle_global_position_s		_global_pos;		/**< global position */
	struct vehicle_land_detected_s			_vehicle_land_detected;	/**< vehicle land detected */
	struct vehicle_rates_setpoint_s			_rates_sp;	/* attitude rates setpoint */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;				/**< if set to true, print debug output */

	float _flaps_applied;
	float _flaperons_applied;


	struct {
		float pssss;

		float p_tc;
		float p_p;

		float pitch_lamda;
		float pitch_n;
		float pitch_rio;
		float pitch_kci;
		float pitch_miu;

		float p_ADRC_timestep;
		float p_ADRC_TD_s;
		float p_ADRC_TD_r;
		float p_ADRC_TD_bd;
		float p_ADRC_NLF_lamda;
		float p_ADRC_NLF_alpha;
		float p_ADRC_NLF_b0;
		float p_ADRC_PD_p;
		float p_ADRC_PD_d;


		float p_i;
		float p_ff;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;
		float r_tc;
		float r_p;
		float r_i;
		float r_ff;
		float r_integrator_max;
		float r_rmax;
		float y_p;
		float y_i;
		float y_ff;
		float y_integrator_max;
		float y_coordinated_min_speed;
		int32_t y_coordinated_method;
		float y_rmax;
		float w_p;
		float w_i;
		float w_ff;
		float w_integrator_max;
		float w_rmax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float trim_roll;
		float trim_pitch;
		float trim_yaw;
		float rollsp_offset_deg;		/**< Roll Setpoint Offset in deg */
		float pitchsp_offset_deg;		/**< Pitch Setpoint Offset in deg */
		float rollsp_offset_rad;		/**< Roll Setpoint Offset in rad */
		float pitchsp_offset_rad;		/**< Pitch Setpoint Offset in rad */
		float man_roll_max;				/**< Max Roll in rad */
		float man_pitch_max;			/**< Max Pitch in rad */
		float man_roll_scale;			/**< scale factor applied to roll actuator control in pure manual mode */
		float man_pitch_scale;			/**< scale factor applied to pitch actuator control in pure manual mode */
		float man_yaw_scale; 			/**< scale factor applied to yaw actuator control in pure manual mode */

		float flaps_scale;				/**< Scale factor for flaps */
		float flaperon_scale;			/**< Scale factor for flaperons */

		int vtol_type;					/**< VTOL type: 0 = tailsitter, 1 = tiltrotor */

		int bat_scale_en;			/**< Battery scaling enabled */

	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t pssss;
		param_t p_ADRC_timestep;
		param_t p_ADRC_TD_s;
		param_t p_ADRC_TD_r;
		param_t p_ADRC_TD_bd;
		param_t p_ADRC_NLF_lamda;
		param_t p_ADRC_NLF_alpha;
		param_t p_ADRC_NLF_b0;
		
		param_t p_ADRC_PD_p;
		param_t p_ADRC_PD_d;
		
		param_t p_tc;
		param_t p_p;

		param_t pitch_lamda;
		param_t pitch_n;
		param_t pitch_rio;
		param_t pitch_kci;
		param_t pitch_miu;


		param_t p_i;
		param_t p_ff;
		param_t p_rmax_pos;
		param_t p_rmax_neg;
		param_t p_integrator_max;
		param_t r_tc;
		param_t r_p;
		param_t r_i;
		param_t r_ff;
		param_t r_integrator_max;
		param_t r_rmax;
		param_t y_p;
		param_t y_i;
		param_t y_ff;
		param_t y_integrator_max;
		param_t y_coordinated_min_speed;
		param_t y_coordinated_method;
		param_t y_rmax;
		param_t w_p;
		param_t w_i;
		param_t w_ff;
		param_t w_integrator_max;
		param_t w_rmax;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;
		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_roll_scale;
		param_t man_pitch_scale;
		param_t man_yaw_scale;

		param_t flaps_scale;
		param_t flaperon_scale;

		param_t vtol_type;

		param_t bat_scale_en;

	}		_parameter_handles;		/**< handles for interesting parameters */

	// Rotation matrix and euler angles to extract from control state
	math::Matrix<3, 3> _R;
	float _roll;
	float _pitch;
	float _yaw;

	float aa;

	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;
	ECL_WheelController			_wheel_ctrl;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Check for global position updates.
	 */
	void		global_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for vehicle land detected updates.
	 */
	void		vehicle_land_detected_poll();

	/**
	 * Check for battery status updates.
	 */
	void		battery_status_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controller collection task.
	 */
	void		task_main();
	void		flap_glipe_control();
	void		stop_motor();
	int 		rw_uart();
	int      		set_uart_baudrate(const int fd, unsigned int baud);
	float yaw_sp_my = 0.0f;
	float roll_sp_my = 0.0f;


};

namespace att_control
{
FixedwingAttitudeControl	*g_control = nullptr;
}

FixedwingAttitudeControl::FixedwingAttitudeControl() :

	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),

	/* subscriptions */
	_accel_sub(-1),
	_att_sp_sub(-1),
	_battery_status_sub(-1),
	_ctrl_state_sub(-1),
	_global_pos_sub(-1),
	_manual_sub(-1),
	_params_sub(-1),
	_vcontrol_mode_sub(-1),
	_vehicle_land_detected_sub(-1),
	_vehicle_status_sub(-1),

	/* publications */
	_rate_sp_pub(nullptr),
	_attitude_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_actuators_2_pub(nullptr),

	_rates_sp_id(nullptr),
	_actuators_id(nullptr),
	_attitude_setpoint_id(nullptr),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fwa_dt")),
#if 0
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fwa_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "fwa_nano")),
#else
	_nonfinite_input_perf(nullptr),
	_nonfinite_output_perf(nullptr),
#endif
	/* states */
	_setpoint_valid(false),
	_debug(false),
	_flaps_applied(0),
	_flaperons_applied(0),
	_roll(0.0f),
	_pitch(0.0f),
	_yaw(0.0f)
{
	/* safely initialize structs */
	_accel = {};
	_actuators = {};
	_actuators_airframe = {};
	_att_sp = {};
	_battery_status = {};
	_ctrl_state = {};
	_global_pos = {};
	_manual = {};
	_rates_sp = {};
	_vcontrol_mode = {};
	_vehicle_land_detected = {};
	_vehicle_status = {};

	_parameter_handles.pssss = param_find("FW_S_S_S_S");

	_parameter_handles.pitch_lamda = param_find("FW_P_MFAC_LAMDA");
	_parameter_handles.pitch_n = param_find("FW_P_MFAC_N");
	_parameter_handles.pitch_rio = param_find("FW_P_MFAC_RIO");
	_parameter_handles.pitch_kci = param_find("FW_P_MFAC_KCI");
	_parameter_handles.pitch_miu = param_find("FW_P_MFAC_MIU");

	_parameter_handles.p_ADRC_timestep = param_find("FW_P_ADRC_T_S");
	_parameter_handles.p_ADRC_TD_s = param_find("FW_P_ADRC_TD_S");
	_parameter_handles.p_ADRC_TD_r = param_find("FW_P_ADRC_TD_R");
	_parameter_handles.p_ADRC_TD_bd = param_find("FW_P_ADRC_BD");
	_parameter_handles.p_ADRC_NLF_lamda = param_find("FW_P_ADRC_NF_LAM");
	_parameter_handles.p_ADRC_NLF_alpha = param_find("FW_P_ADRC_NF_ALP");
	_parameter_handles.p_ADRC_NLF_b0 = param_find("FW_P_ADRC_NF_B0");
	_parameter_handles.p_ADRC_PD_p = param_find("FW_P_ADRC_P");
	_parameter_handles.p_ADRC_PD_d = param_find("FW_P_ADRC_D");

	_parameter_handles.p_tc = param_find("FW_P_TC");
	_parameter_handles.p_p = param_find("FW_PR_P");
	_parameter_handles.p_i = param_find("FW_PR_I");
	_parameter_handles.p_ff = param_find("FW_PR_FF");
	_parameter_handles.p_rmax_pos = param_find("FW_P_RMAX_POS");
	_parameter_handles.p_rmax_neg = param_find("FW_P_RMAX_NEG");
    _parameter_handles.p_integrator_max = param_find("FW_PR_IMAX");

	_parameter_handles.r_tc = param_find("FW_R_TC");
	_parameter_handles.r_p = param_find("FW_RR_P");
	_parameter_handles.r_i = param_find("FW_RR_I");
	_parameter_handles.r_ff = param_find("FW_RR_FF");
	_parameter_handles.r_integrator_max = param_find("FW_RR_IMAX");
	_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_parameter_handles.y_p = param_find("FW_YR_P");
	_parameter_handles.y_i = param_find("FW_YR_I");
	_parameter_handles.y_ff = param_find("FW_YR_FF");
	_parameter_handles.y_integrator_max = param_find("FW_YR_IMAX");
	_parameter_handles.y_rmax = param_find("FW_Y_RMAX");

	_parameter_handles.w_p = param_find("FW_WR_P");
	_parameter_handles.w_i = param_find("FW_WR_I");
	_parameter_handles.w_ff = param_find("FW_WR_FF");
	_parameter_handles.w_integrator_max = param_find("FW_WR_IMAX");
	_parameter_handles.w_rmax = param_find("FW_W_RMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.y_coordinated_min_speed = param_find("FW_YCO_VMIN");
	_parameter_handles.y_coordinated_method = param_find("FW_YCO_METHOD");

	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.man_roll_max = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max = param_find("FW_MAN_P_MAX");
	_parameter_handles.man_roll_scale = param_find("FW_MAN_R_SC");
	_parameter_handles.man_pitch_scale = param_find("FW_MAN_P_SC");
	_parameter_handles.man_yaw_scale = param_find("FW_MAN_Y_SC");

	_parameter_handles.flaps_scale = param_find("FW_FLAPS_SCL");
	_parameter_handles.flaperon_scale = param_find("FW_FLAPERON_SCL");

	_parameter_handles.vtol_type = param_find("VT_TYPE");

	_parameter_handles.bat_scale_en = param_find("FW_BAT_SCALE_EN");

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	att_control::g_control = nullptr;
}

int
FixedwingAttitudeControl::parameters_update()
{

	param_get(_parameter_handles.p_tc, &(_parameters.p_tc));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));

	param_get(_parameter_handles.pitch_lamda,&(_parameters.pitch_lamda));
	param_get(_parameter_handles.pitch_n,&(_parameters.pitch_n));
	param_get(_parameter_handles.pitch_rio,&(_parameters.pitch_rio));
	param_get(_parameter_handles.pitch_kci,&(_parameters.pitch_kci));
	param_get(_parameter_handles.pitch_miu,&(_parameters.pitch_miu));

	param_get(_parameter_handles.p_ADRC_timestep,&(_parameters.p_ADRC_timestep));
	param_get(_parameter_handles.p_ADRC_TD_s,&(_parameters.p_ADRC_TD_s));
	param_get(_parameter_handles.p_ADRC_TD_r,&(_parameters.p_ADRC_TD_r));
	param_get(_parameter_handles.p_ADRC_TD_bd,&(_parameters.p_ADRC_TD_bd));
	param_get(_parameter_handles.p_ADRC_NLF_lamda ,&(_parameters.p_ADRC_NLF_lamda ));
	param_get(_parameter_handles.p_ADRC_NLF_alpha ,&(_parameters.p_ADRC_NLF_alpha ));
	param_get(_parameter_handles.p_ADRC_NLF_b0 ,&(_parameters.p_ADRC_NLF_b0 ));
	param_get(_parameter_handles.p_ADRC_PD_p,&(_parameters.p_ADRC_PD_p));
	param_get(_parameter_handles.p_ADRC_PD_d,&(_parameters.p_ADRC_PD_d));

	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
	param_get(_parameter_handles.p_rmax_pos, &(_parameters.p_rmax_pos));
	param_get(_parameter_handles.p_rmax_neg, &(_parameters.p_rmax_neg));
	param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));

	param_get(_parameter_handles.r_tc, &(_parameters.r_tc));
	param_get(_parameter_handles.pssss,&(_parameters.pssss));
	//param_get(_parameter_handles.pitch_lamda,&(_parameters.pitch_lamda));
	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_ff, &(_parameters.r_ff));

	param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));
	param_get(_parameter_handles.y_ff, &(_parameters.y_ff));
	param_get(_parameter_handles.y_integrator_max, &(_parameters.y_integrator_max));
	param_get(_parameter_handles.y_coordinated_min_speed, &(_parameters.y_coordinated_min_speed));
	param_get(_parameter_handles.y_coordinated_method, &(_parameters.y_coordinated_method));
	param_get(_parameter_handles.y_rmax, &(_parameters.y_rmax));

	param_get(_parameter_handles.w_p, &(_parameters.w_p));
	param_get(_parameter_handles.w_i, &(_parameters.w_i));
	param_get(_parameter_handles.w_ff, &(_parameters.w_ff));
	param_get(_parameter_handles.w_integrator_max, &(_parameters.w_integrator_max));
	param_get(_parameter_handles.w_rmax, &(_parameters.w_rmax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.rollsp_offset_deg, &(_parameters.rollsp_offset_deg));
	param_get(_parameter_handles.pitchsp_offset_deg, &(_parameters.pitchsp_offset_deg));
	_parameters.rollsp_offset_rad = math::radians(_parameters.rollsp_offset_deg);
	_parameters.pitchsp_offset_rad = math::radians(_parameters.pitchsp_offset_deg);
	param_get(_parameter_handles.man_roll_max, &(_parameters.man_roll_max));
	param_get(_parameter_handles.man_pitch_max, &(_parameters.man_pitch_max));
	_parameters.man_roll_max = math::radians(_parameters.man_roll_max);
	_parameters.man_pitch_max = math::radians(_parameters.man_pitch_max);
	param_get(_parameter_handles.man_roll_scale, &(_parameters.man_roll_scale));
	param_get(_parameter_handles.man_pitch_scale, &(_parameters.man_pitch_scale));
	param_get(_parameter_handles.man_yaw_scale, &(_parameters.man_yaw_scale));

	param_get(_parameter_handles.flaps_scale, &_parameters.flaps_scale);
	param_get(_parameter_handles.flaperon_scale, &_parameters.flaperon_scale);

	param_get(_parameter_handles.vtol_type, &_parameters.vtol_type);

	param_get(_parameter_handles.bat_scale_en, &_parameters.bat_scale_en);

	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.p_tc);
	_pitch_ctrl.set_k_p(_parameters.p_p);

	_pitch_ctrl.set_lamda(_parameters.pitch_lamda);
	_pitch_ctrl.set_n(_parameters.pitch_n);
	_pitch_ctrl.set_rio(_parameters.pitch_rio);
	_pitch_ctrl.set_kci(_parameters.pitch_kci);
	_pitch_ctrl.set_miu(_parameters.pitch_miu);

	_pitch_ctrl.set_timestep(_parameters.p_ADRC_timestep);
	_pitch_ctrl.set_s(_parameters.p_ADRC_TD_s);
	_pitch_ctrl.set_r(_parameters.p_ADRC_TD_r);
	_pitch_ctrl.set_bd(_parameters.p_ADRC_TD_bd);
	_pitch_ctrl.set_adrc_lamda(_parameters.p_ADRC_NLF_lamda);
	_pitch_ctrl.set_adrc_alpha(_parameters.p_ADRC_NLF_alpha);
	_pitch_ctrl.set_adrc_b0(_parameters.p_ADRC_NLF_b0);

	_pitch_ctrl.set_p(_parameters.p_ADRC_PD_p);
	_pitch_ctrl.set_d(_parameters.p_ADRC_PD_d);

	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);
	_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
	_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));

	/* roll control parameters */
	aa = _parameters.pssss;
	_roll_ctrl.set_time_constant(_parameters.r_tc);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);
	_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);
	_yaw_ctrl.set_coordinated_min_speed(_parameters.y_coordinated_min_speed);
	_yaw_ctrl.set_coordinated_method(_parameters.y_coordinated_method);
	_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));

	/* wheel control parameters */
	_wheel_ctrl.set_k_p(_parameters.w_p);
	_wheel_ctrl.set_k_i(_parameters.w_i);
	_wheel_ctrl.set_k_ff(_parameters.w_ff);
	_wheel_ctrl.set_integrator_max(_parameters.w_integrator_max);
	_wheel_ctrl.set_max_rate(math::radians(_parameters.w_rmax));

	return PX4_OK;
}

void
FixedwingAttitudeControl::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
FixedwingAttitudeControl::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
FixedwingAttitudeControl::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
FixedwingAttitudeControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
		_setpoint_valid = true;
	}
}

void
FixedwingAttitudeControl::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
FixedwingAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(fw_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_fw);
				_attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_land_detected_poll()
{
	/* check if there is new status information */
	bool vehicle_land_detected_updated;
	orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

	if (vehicle_land_detected_updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}
}

void
FixedwingAttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
FixedwingAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	att_control::g_control->task_main();
}

void
FixedwingAttitudeControl::task_main()
{
	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

/*
	orb_set_interval(_att_sp_sub, _parameter_handles.p_ADRC_timestep*1000);
	orb_set_interval(_ctrl_state_sub, _parameter_handles.p_ADRC_timestep*1000);
	orb_set_interval(_accel_sub, _parameter_handles.p_ADRC_timestep*1000);
	orb_set_interval(_vcontrol_mode_sub, _parameter_handles.p_ADRC_timestep*1000);
	orb_set_interval(_params_sub, _parameter_handles.p_ADRC_timestep*1000);
	orb_set_interval(_manual_sub, _parameter_handles.p_ADRC_timestep*1000);
	orb_set_interval(_global_pos_sub, _parameter_handles.p_ADRC_timestep*1000);
	orb_set_interval(_vehicle_status_sub, _parameter_handles.p_ADRC_timestep*1000);
	orb_set_interval(_vehicle_land_detected_sub, _parameter_handles.p_ADRC_timestep*1000);
	orb_set_interval(_battery_status_sub, _parameter_handles.p_ADRC_timestep*1000);
*/
	orb_set_interval(_att_sp_sub, 10);
	orb_set_interval(_ctrl_state_sub,10);
	orb_set_interval(_accel_sub, 10);
	orb_set_interval(_vcontrol_mode_sub,10);
	orb_set_interval(_params_sub, 10);
	orb_set_interval(_manual_sub, 10);
	orb_set_interval(_global_pos_sub, 10);
	orb_set_interval(_vehicle_status_sub, 10);
	orb_set_interval(_vehicle_land_detected_sub, 10);
	orb_set_interval(_battery_status_sub, 10);
	//printf("ssssss" );
	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_setpoint_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	vehicle_status_poll();
	vehicle_land_detected_poll();
	battery_status_poll();

	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _ctrl_state_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {
		static int loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}
                    
		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();
			//printf("%8.6f\n",(double)deltaT );
			/* guard against too large deltaT's */
			if (deltaT > 1.0f) {
				deltaT = 0.01f;
			}

			/* load local copies */
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);


			/* get current rotation matrix and euler angles from control state quaternions */
			math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
			_R = q_att.to_dcm();

			math::Vector<3> euler_angles;
			euler_angles = _R.to_euler();
			_roll    = euler_angles(0);
			_pitch   = euler_angles(1);
			_yaw     = euler_angles(2);

			if (_vehicle_status.is_vtol && _parameters.vtol_type == 0) {
				/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
				 *
				 * Since the VTOL airframe is initialized as a multicopter we need to
				 * modify the estimated attitude for the fixed wing operation.
				 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
				 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
				 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
				 * Additionally, in order to get the correct sign of the pitch, we need to multiply
				 * the new x axis of the rotation matrix with -1
				 *
				 * original:			modified:
				 *
				 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
				 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
				 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
				 * */
				math::Matrix<3, 3> R_adapted = _R;		//modified rotation matrix

				/* move z to x */
				R_adapted(0, 0) = _R(0, 2);
				R_adapted(1, 0) = _R(1, 2);
				R_adapted(2, 0) = _R(2, 2);

				/* move x to z */
				R_adapted(0, 2) = _R(0, 0);
				R_adapted(1, 2) = _R(1, 0);
				R_adapted(2, 2) = _R(2, 0);

				/* change direction of pitch (convert to right handed system) */
				R_adapted(0, 0) = -R_adapted(0, 0);
				R_adapted(1, 0) = -R_adapted(1, 0);
				R_adapted(2, 0) = -R_adapted(2, 0);
				euler_angles = R_adapted.to_euler();  //adapted euler angles for fixed wing operation

				/* fill in new attitude data */
				_R = R_adapted;
				_roll    = euler_angles(0);
				_pitch   = euler_angles(1);
				_yaw     = euler_angles(2);

				/* lastly, roll- and yawspeed have to be swaped */
				float helper = _ctrl_state.roll_rate;
				_ctrl_state.roll_rate = -_ctrl_state.yaw_rate;
				_ctrl_state.yaw_rate = helper;
			}
			
			vehicle_setpoint_poll();

			vehicle_accel_poll();

			vehicle_control_mode_poll();

			vehicle_manual_poll();

			global_pos_poll();

			vehicle_status_poll();

			vehicle_land_detected_poll();

			battery_status_poll();
			
			// the position controller will not emit attitude setpoints in some modes
			// we need to make sure that this flag is reset
			_att_sp.fw_control_yaw = _att_sp.fw_control_yaw && _vcontrol_mode.flag_control_auto_enabled;

			/* lock integrator until control is started */
			bool lock_integrator;

			if (_vcontrol_mode.flag_control_attitude_enabled && !_vehicle_status.is_rotary_wing) {
				lock_integrator = false;

			} else {
				lock_integrator = true;
			}

			/* Simple handling of failsafe: deploy parachute if failsafe is on */
			if (_vcontrol_mode.flag_control_termination_enabled) {
				_actuators_airframe.control[7] = 1.0f;
				//warnx("_actuators_airframe.control[1] = 1.0f;");

			} else {
				_actuators_airframe.control[7] = 0.0f;
				//warnx("_actuators_airframe.control[1] = -1.0f;");
			}

			/* if we are in rotary wing mode, do nothing */
			if (_vehicle_status.is_rotary_wing && !_vehicle_status.is_vtol) {
				continue;
			}

			/* default flaps to center */
			float flap_control = 0.0f;

			/* map flaps by default to manual if valid */
			if (PX4_ISFINITE(_manual.flaps) && _vcontrol_mode.flag_control_manual_enabled
			    && fabsf(_parameters.flaps_scale) > 0.01f) {
				flap_control = 0.5f * (_manual.flaps + 1.0f) * _parameters.flaps_scale;

			} else if (_vcontrol_mode.flag_control_auto_enabled
				   && fabsf(_parameters.flaps_scale) > 0.01f) {
				flap_control = _att_sp.apply_flaps ? 1.0f * _parameters.flaps_scale : 0.0f;
			}

			// move the actual control value continuous with time, full flap travel in 1sec
			if (fabsf(_flaps_applied - flap_control) > 0.01f) {
				_flaps_applied += (_flaps_applied - flap_control) < 0 ? deltaT : -deltaT;

			} else {
				_flaps_applied = flap_control;
			}

			/* default flaperon to center */
			float flaperon_control = 0.0f;

			/* map flaperons by default to manual if valid */
			if (PX4_ISFINITE(_manual.aux2) && _vcontrol_mode.flag_control_manual_enabled
			    && fabsf(_parameters.flaperon_scale) > 0.01f) {
				flaperon_control = 0.5f * (_manual.aux2 + 1.0f) * _parameters.flaperon_scale;

			} else if (_vcontrol_mode.flag_control_auto_enabled
				   && fabsf(_parameters.flaperon_scale) > 0.01f) {
				flaperon_control = _att_sp.apply_flaps ? 1.0f * _parameters.flaperon_scale : 0.0f;
			}

			// move the actual control value continuous with time, full flap travel in 1sec
			if (fabsf(_flaperons_applied - flaperon_control) > 0.01f) {
				_flaperons_applied += (_flaperons_applied - flaperon_control) < 0 ? deltaT : -deltaT;

			} else {
				_flaperons_applied = flaperon_control;
			}
			
			/* decide if in stabilized or full manual control */
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				/* scale around tuning airspeed */
			//if (1) {
				float airspeed;

				/* if airspeed is not updating, we assume the normal average speed */
				if (bool nonfinite = !PX4_ISFINITE(_ctrl_state.airspeed) || !_ctrl_state.airspeed_valid) {
					airspeed = _parameters.airspeed_trim;

					if (nonfinite) {
						perf_count(_nonfinite_input_perf);
					}

				} else {
					/* prevent numerical drama by requiring 0.5 m/s minimal speed */
					airspeed = math::max(0.5f, _ctrl_state.airspeed);
				}

				/*
				 * For scaling our actuators using anything less than the min (close to stall)
				 * speed doesn't make any sense - its the strongest reasonable deflection we
				 * want to do in flight and its the baseline a human pilot would choose.
				 *
				 * Forcing the scaling to this value allows reasonable handheld tests.
				 */
				float airspeed_scaling = _parameters.airspeed_trim / ((airspeed < _parameters.airspeed_min) ? _parameters.airspeed_min :
							 airspeed);

				/* Use min airspeed to calculate ground speed scaling region.
				 * Don't scale below gspd_scaling_trim
				 */
				float groundspeed = sqrtf(_global_pos.vel_n * _global_pos.vel_n +
							  _global_pos.vel_e * _global_pos.vel_e);
				float gspd_scaling_trim = (_parameters.airspeed_min * 0.6f);
				float groundspeed_scaler = gspd_scaling_trim / ((groundspeed < gspd_scaling_trim) ? gspd_scaling_trim : groundspeed);

				float roll_sp = _parameters.rollsp_offset_rad;
				float pitch_sp = _parameters.pitchsp_offset_rad;
				float yaw_sp = 0.0f;
				float yaw_manual = 0.0f;
				float throttle_sp = 0.0f;

				// in STABILIZED mode we need to generate the attitude setpoint
				// from manual user inputs
				if (!_vcontrol_mode.flag_control_climb_rate_enabled) {
					_att_sp.roll_body = _manual.y * _parameters.man_roll_max + _parameters.rollsp_offset_rad;
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, -_parameters.man_roll_max, _parameters.man_roll_max);
					_att_sp.pitch_body = -_manual.x * _parameters.man_pitch_max + _parameters.pitchsp_offset_rad;
					_att_sp.pitch_body = math::constrain(_att_sp.pitch_body, -_parameters.man_pitch_max, _parameters.man_pitch_max);
					_att_sp.yaw_body = 0.0f;
					_att_sp.thrust = _manual.z;
					int instance;
					orb_publish_auto(_attitude_setpoint_id, &_attitude_sp_pub, &_att_sp, &instance, ORB_PRIO_DEFAULT);
				}
				
				roll_sp = _att_sp.roll_body;
				pitch_sp = _att_sp.pitch_body;
				yaw_sp = _att_sp.yaw_body;
				throttle_sp = _att_sp.thrust;

				/* allow manual yaw in manual modes */
				if (_vcontrol_mode.flag_control_manual_enabled) {
					yaw_manual = _manual.r;
				}

				/* reset integrals where needed */
				if (_att_sp.roll_reset_integral) {
					_roll_ctrl.reset_integrator();
				}

				if (_att_sp.pitch_reset_integral) {
					_pitch_ctrl.reset_integrator();
				}

				if (_att_sp.yaw_reset_integral) {
					_yaw_ctrl.reset_integrator();
					_wheel_ctrl.reset_integrator();
				}

				/* If the aircraft is on ground reset the integrators */
				if (_vehicle_land_detected.landed || _vehicle_status.is_rotary_wing) {
					_roll_ctrl.reset_integrator();
					_pitch_ctrl.reset_integrator();
					_yaw_ctrl.reset_integrator();
					_wheel_ctrl.reset_integrator();
				}

				/* Prepare speed_body_u and speed_body_w */
				float speed_body_u = _R(0, 0) * _global_pos.vel_n + _R(1, 0) * _global_pos.vel_e + _R(2, 0) * _global_pos.vel_d;
				float speed_body_v = _R(0, 1) * _global_pos.vel_n + _R(1, 1) * _global_pos.vel_e + _R(2, 1) * _global_pos.vel_d;
				float speed_body_w = _R(0, 2) * _global_pos.vel_n + _R(1, 2) * _global_pos.vel_e + _R(2, 2) * _global_pos.vel_d;

				/* Prepare data for attitude controllers */
				struct ECL_ControlData control_input = {};
				control_input.roll = _roll;
				control_input.pitch = _pitch;
				control_input.yaw = _yaw;
				control_input.roll_rate = _ctrl_state.roll_rate;
				control_input.pitch_rate = _ctrl_state.pitch_rate;
				control_input.yaw_rate = _ctrl_state.yaw_rate;
				control_input.speed_body_u = speed_body_u;
				control_input.speed_body_v = speed_body_v;
				control_input.speed_body_w = speed_body_w;
				control_input.acc_body_x = _accel.x;
				control_input.acc_body_y = _accel.y;
				control_input.acc_body_z = _accel.z;
				control_input.roll_setpoint = roll_sp;
				control_input.pitch_setpoint = pitch_sp;
				control_input.yaw_setpoint = yaw_sp;

				roll_sp_my = roll_sp;
				yaw_sp_my = yaw_sp;

				control_input.airspeed_min = _parameters.airspeed_min;
				control_input.airspeed_max = _parameters.airspeed_max;
				control_input.airspeed = airspeed;
				control_input.scaler = airspeed_scaling;
				control_input.lock_integrator = lock_integrator;
				control_input.groundspeed = groundspeed;
				control_input.groundspeed_scaler = groundspeed_scaler;

				_yaw_ctrl.set_coordinated_method(_parameters.y_coordinated_method);
				
				/* Run attitude controllers */
				if (PX4_ISFINITE(roll_sp) && PX4_ISFINITE(pitch_sp)) {
					_roll_ctrl.control_attitude(control_input);
					_pitch_ctrl.control_attitude(control_input);
					_yaw_ctrl.control_attitude(control_input); //runs last, because is depending on output of roll and pitch attitude
					_wheel_ctrl.control_attitude(control_input);

					/* Update input data for rate controllers */
					control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
					control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
					control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

					/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
					float roll_u = _roll_ctrl.control_bodyrate(control_input);
					_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + _parameters.trim_roll :
							_parameters.trim_roll;

					if (!PX4_ISFINITE(roll_u)) {
						_roll_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);

						if (_debug && loop_counter % 10 == 0) {
							warnx("roll_u %.4f", (double)roll_u);
						}
					}

					//float pitch_u = _pitch_ctrl.control_bodyrate(control_input);  //(-1~1)
					//PID control end here
					//MFAC start 
					//printf("%8.4f\n",(double)pitch_u );

					float pitch_u_MFAC =_pitch_ctrl.MFAC_control(control_input);

					static float pitch_u_ADRC = 0.0f;
					pitch_u_ADRC = _pitch_ctrl.ADRC_control(control_input,pitch_u_ADRC);
					printf("pitch_u_ADRC = %8.4f\n",(double)pitch_u_ADRC );
					//printf("%lf\n",(double)pitch_u_MFAC );
					//struct ADRC_s ADRC_log;
					//orb_advert_t ADRC_sub = orb_advertise(ORB_ID(ADRC), &ADRC_log);
					//ADRC_log.x1 = 0.48;
					//orb_publish(ORB_ID(ADRC), ADRC_sub, &ADRC_log);
					//MFAC end

					_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u_ADRC)) ? pitch_u_ADRC + _parameters.trim_pitch :
							_parameters.trim_pitch;
					//printf("%8.6f\n",(double)_actuators.control[actuator_controls_s::INDEX_PITCH] );
					//printf("%8.6f\n",(double)_parameters.trim_pitch);
							//output  Result

					if (!PX4_ISFINITE(pitch_u_MFAC)) {
						_pitch_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);

						if (_debug && loop_counter % 10 == 0) {
							warnx("pitch_u_MFAC %.4f, _yaw_ctrl.get_desired_rate() %.4f,"
							      " airspeed %.4f, airspeed_scaling %.4f,"
							      " roll_sp %.4f, pitch_sp %.4f,"
							      " _roll_ctrl.get_desired_rate() %.4f,"
							      " _pitch_ctrl.get_desired_rate() %.4f"
							      " att_sp.roll_body %.4f",
							      (double)pitch_u_MFAC, (double)_yaw_ctrl.get_desired_rate(),
							      (double)airspeed, (double)airspeed_scaling,
							      (double)roll_sp, (double)pitch_sp,
							      (double)_roll_ctrl.get_desired_rate(),
							      (double)_pitch_ctrl.get_desired_rate(),
							      (double)_att_sp.roll_body);
						}
					}

					float yaw_u = 0.0f;
					
					if (_att_sp.fw_control_yaw == true) {
						yaw_u = _wheel_ctrl.control_bodyrate(control_input);

					} else {
						yaw_u = _yaw_ctrl.control_bodyrate(control_input);
					}

					_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + _parameters.trim_yaw :
							_parameters.trim_yaw;

					/* add in manual rudder control */
					_actuators.control[actuator_controls_s::INDEX_YAW] += yaw_manual;

					if (!PX4_ISFINITE(yaw_u)) {
						_yaw_ctrl.reset_integrator();
						_wheel_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);

						if (_debug && loop_counter % 10 == 0) {
							warnx("yaw_u %.4f", (double)yaw_u);
						}
					}

					/* throttle passed through if it is finite and if no engine failure was detected */
					/*
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(throttle_sp) &&
							!(_vehicle_status.engine_failure ||
							  _vehicle_status.engine_failure_cmd)) ?
							throttle_sp : 0.0f;
					*/
					//在自驾状态下，油门通道通过遥控器控制
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;

					
					/* scale effort by battery status */
					if (_parameters.bat_scale_en && _battery_status.scale > 0.0f &&
					    _actuators.control[actuator_controls_s::INDEX_THROTTLE] > 0.1f) {
						_actuators.control[actuator_controls_s::INDEX_THROTTLE] *= _battery_status.scale;
					}


					if (!PX4_ISFINITE(throttle_sp)) {
						if (_debug && loop_counter % 10 == 0) {
							warnx("throttle_sp %.4f", (double)throttle_sp);
						}
					}

				} else {
					perf_count(_nonfinite_input_perf);

					if (_debug && loop_counter % 10 == 0) {
						warnx("Non-finite setpoint roll_sp: %.4f, pitch_sp %.4f", (double)roll_sp, (double)pitch_sp);
					}
				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				_rates_sp.roll = _roll_ctrl.get_desired_rate();
				_rates_sp.pitch = _pitch_ctrl.get_desired_rate();
				_rates_sp.yaw = _yaw_ctrl.get_desired_rate();

				_rates_sp.timestamp = hrt_absolute_time();

				if (_rate_sp_pub != nullptr) {
					/* publish the attitude rates setpoint */
					orb_publish(_rates_sp_id, _rate_sp_pub, &_rates_sp);

				} else if (_rates_sp_id) {
					/* advertise the attitude rates setpoint */
					_rate_sp_pub = orb_advertise(_rates_sp_id, &_rates_sp);
				}

			} else {
				/* manual/direct control */
				_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y * _parameters.man_roll_scale + _parameters.trim_roll;
				_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x * _parameters.man_pitch_scale +
						_parameters.trim_pitch;
				_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _parameters.man_yaw_scale + _parameters.trim_yaw;
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
			}

			_actuators.control[actuator_controls_s::INDEX_FLAPS] = _flaps_applied;
			_actuators.control[5] = _manual.aux1;
			_actuators.control[actuator_controls_s::INDEX_AIRBRAKES] = _flaperons_applied;
			// FIXME: this should use _vcontrol_mode.landing_gear_pos in the future
			_actuators.control[7] = _manual.aux3;

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _ctrl_state.timestamp;
			_actuators_airframe.timestamp = hrt_absolute_time();
			_actuators_airframe.timestamp_sample = _ctrl_state.timestamp;

			//_actuators.control[3] = 30.0f; 
			//flap_glipe_control();
			//rw_uart();
			//printf("_actuators.control[3] = \t%lf\n",(double)_actuators.control[3]);

			/* Only publish if any of the proper modes are enabled */
			if (_vcontrol_mode.flag_control_rates_enabled ||
			    _vcontrol_mode.flag_control_attitude_enabled ||
			    _vcontrol_mode.flag_control_manual_enabled) {
				/* publish the actuator controls */
				if (_actuators_0_pub != nullptr) {
					orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

				} else if (_actuators_id) {
					_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
				}

				if (_actuators_2_pub != nullptr) {
					/* publish the actuator controls*/
					orb_publish(ORB_ID(actuator_controls_2), _actuators_2_pub, &_actuators_airframe);

				} else {
					/* advertise and publish */
					_actuators_2_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators_airframe);
				}
			}

		//printf("aaaaaa" );
		}
		//PX4_INFO("_actuators.control[0] = \t%8.4f",(double)_actuators.control[0]);
		//PX4_INFO("_actuators.control[1] = \t%8.4f",(double)_actuators.control[1]);
		//PX4_INFO("_actuators.control[2] = \t%8.4f",(double)_actuators.control[2]);
		//PX4_INFO("_actuators.control[3] = \t%8.4f",(double)_actuators.control[3]);
		//PX4_INFO("_actuators.timestamp_sample = \t%8.4f",(double)_actuators.timestamp_sample);
		loop_counter++;
		perf_end(_loop_perf);
	}
	
	PX4_INFO("OUT");
	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
}



int FixedwingAttitudeControl::set_uart_baudrate(const int fd, unsigned int baud)
{
	int speed;

	switch (baud)
	{
		case 9600:          speed = B9600;       break;
		case 19200:        speed = B19200;     break;
		case 38400:        speed = B38400;     break;
		case 57600:        speed = B57600;     break;
		case 115200:      speed = B115200;   break;
		default:
		//warnx("ERR:  baudrate:  %d\n", baud);

		return -EINVAL;
	}


	struct termios uart_config;

	int termios_state;

	// fill the struct for the new configuration*/
	tcgetattr(fd, &uart_config);
             // clear ONLCR flag (which appends a CR for every LF)*/
	uart_config.c_oflag &= ~ONLCR;
	//no parity, one stop bit*/
	uart_config.c_cflag &= ~(CSTOPB|PARENB);
	//set baud rate*/
	if ((termios_state = cfsetispeed (&uart_config, speed)) < 0)
	{
		warnx("ERR: %d (cfsetispeed) \n", termios_state);
		return false;
	}

	if ((termios_state = tcsetattr (fd, TCSANOW,  &uart_config)) < 0)
	{
		warnx("ERR: %d (tcsetattr) \n", termios_state);
		return false;
	}

	return true;
}

int
FixedwingAttitudeControl::rw_uart(void)
{
	

	char data[] = "D1234567812345678\t";
	//char buffer[]= "";
	//char write_dataF[4] = "FFFF";
	//char write_dataL[4] = "LLLL";
	//int j = 0;

	int uart_read = 1;
	//open("/dev/ttyS6", O_RDWR|O_NOCTTY);  //kai chuan kou 

	if (uart_read < 0)
	{
		err(1, "failed to open port: %s", "/dev/ttyS6");
		return -1;
	}
	
	
	if (false == set_uart_baudrate(uart_read, 9600))
	{
		printf("[YCM]set_uart_baudrate is failed \n");
		return -1; 
	}
	printf("[YSM] uart init is successful\n");
	
	// while(1)
	// 	{printf("OK" );}
	
	while (true)
	{
		printf("size = %d\n",sizeof(data));
		int aaa = write(uart_read, &data, sizeof(data));
		printf("aaa = %d\n",aaa );
		sleep(1);

		//int bbb = read(uart_read,&buffer,3);
		//printf("bbb = %d\n",bbb );



		//if(data == 'R')
		// //{
		// 	for(int i = 0; i < 4; ++i )
		// 	{
		// 		read(uart_read, &data, 1); // du shu ju
		// 		buffer[i] = data;
		// 		data = '0';
		// 	}
			
		// 	printf("%s\n",buffer );
			
			//write(uart_read, &write_dataF, (size_t)write_dataF);
		//if(data == 'L')
			//write(uart_read, &write_dataL, (size_t)write_dataL);
			//sleep(10);
			//break;

		//}

		//write(uart_read, &write_data, (size_t)write_data);
		//if(j>100)
		//	break;

			
		//printf("IN ");

	//}
	}
	
	
	//printf("OUT");
	return 0;
}



void
FixedwingAttitudeControl::flap_glipe_control(void)
{
	//PX4_INFO("_actuators.control[3] = \t%8.4f",(double)_actuators.control[3]);
	/* if */	
	//PX4_INFO(" _vcontrol_mode.flag_control_auto_enabled = %8.4f", (double)_vcontrol_mode.flag_control_manual_enabled);
	//PX4_INFO("_yaw =\t %8.4f",(double)_yaw);
	//PX4_INFO("_roll = \t%8.4f",(double)_roll);
	//PX4_INFO("yaw_sp =\t%8.4f",(double)yaw_sp_my);
	//PX4_INFO("roll_sp =\t%8.4f",(double)roll_sp_my);
	/*if( _vcontrol_mode.flag_control_manual_enabled)
	{
		if(fabsf(_roll)<0.6f)
		{
			//continue;

		}
		else
		{
			stop_motor();
		}
	}*/

	static float last_time = 0.0f;
	static float all_time = 0.0f; 

	static  float temp_control = _actuators.control[3];

	float now_time =  hrt_absolute_time();

	float delta_time = now_time - last_time;
	last_time = now_time;
	all_time = all_time + delta_time;
	
	//PX4_INFO("_manual.x = :\t%8.4f", (double)_manual.flaps);
	if(_vcontrol_mode.flag_control_auto_enabled&&fabsf(_roll)>0.3f)
	{
		
	//printf("last_time = %lf\n",(double)last_time);
	//printf("delta_time = %lf\n",(double)delta_time);
	//printf("all_time = \t%lf\n",(double)all_time);
	
		if(all_time>=330000)     //0.33s
		{
		//printf("all_time = \t%lf\n",(double)all_time);
		_actuators.control[3] = temp_control/3;
		temp_control = temp_control/3;
		
		all_time = 0.0f;
		
		}
		else
		{
			_actuators.control[3] = temp_control;
		}

		

	//printf("now_time = \t%8.4f\n",(double)now_time );
	}
	if(fabsf(_roll)<0.3f)
		{
			temp_control = _actuators.control[3];


		}
	}

void
FixedwingAttitudeControl::stop_motor(void)
{
	
	static float last_time = 0.0f;
	static float all_time = 0.0f; 

	static  float temp_control = _actuators.control[3];

	float now_time =  hrt_absolute_time();

	float delta_time = now_time - last_time;
	last_time = now_time;
	all_time = all_time + delta_time;
	//printf("last_time = %lf\n",(double)last_time);
	//printf("delta_time = %lf\n",(double)delta_time);
	//printf("all_time = \t%lf\n",(double)all_time);
	
		if(all_time>=330000)     //0.33s
		{
		//printf("all_time = \t%lf\n",(double)all_time);
		_actuators.control[3] = temp_control/3;
		temp_control = temp_control/3;
		
		all_time = 0.0f;
		
		}
		else
		{
			_actuators.control[3] = temp_control;
		}

		if(fabsf(_roll)<0.6f)
		{
			temp_control = _actuators.control[3];
		}
	//printf("now_time = \t%8.4f\n",(double)now_time );
}

int
FixedwingAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("fw_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1400,
					   (px4_main_t)&FixedwingAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return PX4_OK;
}

int fw_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: fw_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		att_control::g_control = new FixedwingAttitudeControl;

		if (att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (PX4_OK != att_control::g_control->start()) {
			delete att_control::g_control;
			att_control::g_control = nullptr;
			warn("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (att_control::g_control == nullptr || !att_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (att_control::g_control == nullptr || !att_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete att_control::g_control;
		att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}



