/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include "ecl_pitch_controller.h"
#include "ecl_roll_controller.h"
#include "ecl_wheel_controller.h"
#include "ecl_yaw_controller.h"
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/juan_attitude_variables.h> //JUAN
#include <uORB/topics/vehicle_local_position_setpoint.h> //JUAN
#include <uORB/topics/vehicle_local_position.h> //JUAN
#include <uORB/topics/wind.h> //JACKSON
#include <uORB/topics/airspeed.h> //JACKSON


using matrix::Eulerf;
using matrix::Quatf;

using uORB::SubscriptionData;

using namespace time_literals;

class FixedwingAttitudeControl final : public ModuleBase<FixedwingAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	FixedwingAttitudeControl(bool vtol = false);
	~FixedwingAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};	/**< vehicle attitude */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};		/**< vehicle attitude setpoint */
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};			/**< battery status subscription */
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};		/**< local position subscription */
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};		/**< notification of manual control updates */
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};		/**< vehicle rates setpoint */
	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};

	//JACKSON
	uORB::Subscription _wind_sub{ORB_ID(wind)};

	//JUAN
	uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _juan_attitude_variables_sub{ORB_ID(juan_attitude_variables)}; //JUAN

	uORB::SubscriptionData<airspeed_s> _airspeed_sub{ORB_ID(airspeed)};

	uORB::Publication<actuator_controls_s>		_actuators_2_pub{ORB_ID(actuator_controls_2)};		/**< actuator control group 1 setpoint (Airframe) */
	uORB::Publication<vehicle_rates_setpoint_s>	_rate_sp_pub{ORB_ID(vehicle_rates_setpoint)};		/**< rate setpoint publication */
	uORB::PublicationMulti<rate_ctrl_status_s>	_rate_ctrl_status_pub{ORB_ID(rate_ctrl_status)};	/**< rate controller status publication */
	// JUAN
	uORB::Publication<vehicle_local_position_setpoint_s>	_local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<juan_attitude_variables_s>	_juan_attitude_variables_pub{ORB_ID(juan_attitude_variables)}; //JUAN

	uORB::SubscriptionData<airspeed_validated_s> _airspeed_validated_sub{ORB_ID(airspeed_validated)};

	uORB::Publication<actuator_controls_s>		_actuators_0_pub;
	uORB::Publication<vehicle_attitude_setpoint_s>	_attitude_sp_pub;

	actuator_controls_s			_actuators {};		/**< actuator control inputs */
	manual_control_setpoint_s		_manual_control_setpoint {};		/**< r/c channel data */
	vehicle_attitude_setpoint_s		_att_sp {};		/**< vehicle attitude setpoint */
	vehicle_control_mode_s			_vcontrol_mode {};	/**< vehicle control mode */
	vehicle_local_position_s		_local_pos {};		/**< local position */
	vehicle_rates_setpoint_s		_rates_sp {};		/* attitude rates setpoint */
	vehicle_status_s			_vehicle_status {};	/**< vehicle status */

	//JACKSON
	wind_s				_wind {};		/**< wind */

	//JUAN
	vehicle_local_position_setpoint_s _local_pos_sp{}; 		//local position setpoint
	juan_attitude_variables_s _juan_att_var{}; 			// JUAN custom attitude control variables

	perf_counter_t	_loop_perf;					/**< loop performance counter */

	hrt_abstime _last_run{0};

	float _flaps_applied{0.0f};
	float _flaperons_applied{0.0f};

	float _airspeed_scaling{1.0f};

	bool _landed{true};

	float _battery_scale{1.0f};

	bool _flag_control_attitude_enabled_last{false};

	bool _is_tailsitter{false};

	//JUAN custom variables
	float _previous_yaw{0.0f};


	// for attitude maneuvers
	float _yaw_test_profile{0.0f};
	float _roll_test_profile{0.0f};
	float _pitch_test_profile{0.0f};

	float _yaw_rate_reference{0.0f};
	float _pitch_rate_reference{0.0f};
	float _roll_rate_reference{0.0f};


	// other variables
	float _previous_time{0.0f};
	float _ground_velocity_corrected{5.0f};
	float _time_elapsed{0.0f};
	float _delta_time_attitude{0.0f};
	float _e_int_1{0.0f};
	float _e_int_2{0.0f};
	float _e_int_3{0.0f};

	// position control variables
	float _pos_x_ref{0.0f}; // position references
	float _pos_y_ref{0.0f};
	float _pos_z_ref{0.0f};
	float _vel_x_ref{0.0f}; // velocity references
	float _vel_y_ref{0.0f};
	float _vel_z_ref{0.0f};
	float _initial_heading{0.0f}; // initialization values
	float _pos_x_initial{0.0f};
	float _pos_y_initial{0.0f};
	float _pos_z_initial{0.0f};
	float _initial_vxy{0.0f};
	float _error_x_int{0.0f};
	float _error_y_int{0.0f};
	float _error_z_int{0.0f};

	// Position controller outputs: ref. DCM and thrust command.
	float C_reference_rows[9] = {1.0f, 0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,1.0f};
	float ThrustN{0.0f};
	int _control_operation_mode{0}; // controller exception management
	float belly_n_old;
	float belly_e_old;
	matrix::Dcmf C_bi;
	matrix::Dcmf C_ri_pos;
	float _error_heading_int{0.0f};
	int _JUAN_flight_mode{1};
	matrix::Vector3f _omega_reference_body;
	float _throttle_out;
	matrix::Dcmf C_ri;
	float _previous_rpm{900.0f};
	float _advance_ratio{0.5f};
	matrix::Vector3f _alpha_reference_body;
	float _global_jar;

	/* ------ Jackson's stuff -----*/
	float t_last = 0.0f;

	float t_last_vertex{0.0f};

	float _acc_x_ref{0.0f};
	float _acc_y_ref{0.0f};
	float fv1{0.0f};
	float fv2{0.0f};

	//Additional thrust
	bool thrust_add_flag{true};
	float T_add{0.0f};

	float _pos_x_last_vtx{0.0f}; //Positions of last vertex reletive to _pos_init
	float _pos_y_last_vtx{0.0f};
	int turnCount{0}; //Number of turns of the box performed
	bool completeFlag = false; //flag showing that the desired path is complete
	bool exitMsgSent = false;
	float t_circ = 0.0f;
	float _pos_x_exit = 0.0f;
	float _pos_y_exit = 0.0f;
	matrix::Dcmf R_wind; //Feedforward rotation
	matrix::Dcmf R_roll; //Roll matrix


	bool feedforward_flag = false; //If true wind feedforward on position is enabled
	bool longTurn = true;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_ACRO_X_MAX>) _param_fw_acro_x_max,
		(ParamFloat<px4::params::FW_ACRO_Y_MAX>) _param_fw_acro_y_max,
		(ParamFloat<px4::params::FW_ACRO_Z_MAX>) _param_fw_acro_z_max,

		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamInt<px4::params::FW_ARSP_MODE>) _param_fw_arsp_mode,

		(ParamInt<px4::params::FW_ARSP_SCALE_EN>) _param_fw_arsp_scale_en,

		(ParamBool<px4::params::FW_BAT_SCALE_EN>) _param_fw_bat_scale_en,

		(ParamFloat<px4::params::FW_DTRIM_P_FLPS>) _param_fw_dtrim_p_flps,
		(ParamFloat<px4::params::FW_DTRIM_P_VMAX>) _param_fw_dtrim_p_vmax,
		(ParamFloat<px4::params::FW_DTRIM_P_VMIN>) _param_fw_dtrim_p_vmin,
		(ParamFloat<px4::params::FW_DTRIM_R_FLPS>) _param_fw_dtrim_r_flps,
		(ParamFloat<px4::params::FW_DTRIM_R_VMAX>) _param_fw_dtrim_r_vmax,
		(ParamFloat<px4::params::FW_DTRIM_R_VMIN>) _param_fw_dtrim_r_vmin,
		(ParamFloat<px4::params::FW_DTRIM_Y_VMAX>) _param_fw_dtrim_y_vmax,
		(ParamFloat<px4::params::FW_DTRIM_Y_VMIN>) _param_fw_dtrim_y_vmin,

		(ParamFloat<px4::params::FW_FLAPERON_SCL>) _param_fw_flaperon_scl,
		(ParamFloat<px4::params::FW_FLAPS_LND_SCL>) _param_fw_flaps_lnd_scl,
		(ParamFloat<px4::params::FW_FLAPS_SCL>) _param_fw_flaps_scl,
		(ParamFloat<px4::params::FW_FLAPS_TO_SCL>) _param_fw_flaps_to_scl,

		(ParamFloat<px4::params::FW_MAN_P_MAX>) _param_fw_man_p_max,
		(ParamFloat<px4::params::FW_MAN_P_SC>) _param_fw_man_p_sc,
		(ParamFloat<px4::params::FW_MAN_R_MAX>) _param_fw_man_r_max,
		(ParamFloat<px4::params::FW_MAN_R_SC>) _param_fw_man_r_sc,
		(ParamFloat<px4::params::FW_MAN_Y_SC>) _param_fw_man_y_sc,

		(ParamFloat<px4::params::FW_P_RMAX_NEG>) _param_fw_p_rmax_neg,
		(ParamFloat<px4::params::FW_P_RMAX_POS>) _param_fw_p_rmax_pos,
		(ParamFloat<px4::params::FW_P_TC>) _param_fw_p_tc,
		(ParamFloat<px4::params::FW_PR_FF>) _param_fw_pr_ff,
		(ParamFloat<px4::params::FW_PR_I>) _param_fw_pr_i,
		(ParamFloat<px4::params::FW_PR_IMAX>) _param_fw_pr_imax,
		(ParamFloat<px4::params::FW_PR_P>) _param_fw_pr_p,
		(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,

		(ParamFloat<px4::params::FW_R_RMAX>) _param_fw_r_rmax,
		(ParamFloat<px4::params::FW_R_TC>) _param_fw_r_tc,
		(ParamFloat<px4::params::FW_RLL_TO_YAW_FF>) _param_fw_rll_to_yaw_ff,
		(ParamFloat<px4::params::FW_RR_FF>) _param_fw_rr_ff,
		(ParamFloat<px4::params::FW_RR_I>) _param_fw_rr_i,
		(ParamFloat<px4::params::FW_RR_IMAX>) _param_fw_rr_imax,
		(ParamFloat<px4::params::FW_RR_P>) _param_fw_rr_p,

		(ParamBool<px4::params::FW_W_EN>) _param_fw_w_en,
		(ParamFloat<px4::params::FW_W_RMAX>) _param_fw_w_rmax,
		(ParamFloat<px4::params::FW_WR_FF>) _param_fw_wr_ff,
		(ParamFloat<px4::params::FW_WR_I>) _param_fw_wr_i,
		(ParamFloat<px4::params::FW_WR_IMAX>) _param_fw_wr_imax,
		(ParamFloat<px4::params::FW_WR_P>) _param_fw_wr_p,

		(ParamFloat<px4::params::FW_Y_RMAX>) _param_fw_y_rmax,
		(ParamFloat<px4::params::FW_YR_FF>) _param_fw_yr_ff,
		(ParamFloat<px4::params::FW_YR_I>) _param_fw_yr_i,
		(ParamFloat<px4::params::FW_YR_IMAX>) _param_fw_yr_imax,
		(ParamFloat<px4::params::FW_YR_P>) _param_fw_yr_p,

		(ParamFloat<px4::params::TRIM_PITCH>) _param_trim_pitch,
		(ParamFloat<px4::params::TRIM_ROLL>) _param_trim_roll,
		(ParamFloat<px4::params::TRIM_YAW>) _param_trim_yaw
	)

	ECL_RollController		_roll_ctrl;
	ECL_PitchController		_pitch_ctrl;
	ECL_YawController		_yaw_ctrl;
	ECL_WheelController		_wheel_ctrl;

	void control_flaps(const float dt);

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	void		vehicle_control_mode_poll();
	void		vehicle_manual_poll();
	void		vehicle_attitude_setpoint_poll();
	void		vehicle_rates_setpoint_poll();
	void		vehicle_land_detected_poll();

	float 		get_airspeed_and_update_scaling();

			//JACKSON'S poll fn
	void		wind_estimate_poll();
	void 		wind_ff_rot_update();

	//JUAN additional functions
	void    	JUAN_position_control();
	void    	JUAN_reference_generator(int _maneuver_type);
	float   	saturate(float value, float min, float max);
	void    	JUAN_singularity_management(float xy_speed, float angle_vect);
};
