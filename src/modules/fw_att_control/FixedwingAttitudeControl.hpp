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

#include <px4_module.h>
#include <drivers/drv_hrt.h>
#include <ecl/attitude_fw/ecl_pitch_controller.h>
#include <ecl/attitude_fw/ecl_roll_controller.h>
#include <ecl/attitude_fw/ecl_wheel_controller.h>
#include <ecl/attitude_fw/ecl_yaw_controller.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/juan_attitude_variables.h> //JUAN
#include <uORB/topics/vehicle_local_position_setpoint.h> //JUAN
#include <uORB/topics/vehicle_local_position.h> //JUAN

using matrix::Eulerf;
using matrix::Quatf;

using uORB::SubscriptionData;

class FixedwingAttitudeControl final : public ModuleBase<FixedwingAttitudeControl>, public px4::WorkItem
{
public:
	FixedwingAttitudeControl();
	~FixedwingAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:

	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};	/**< vehicle attitude */

	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};		/**< vehicle attitude setpoint */
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};			/**< battery status subscription */
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};		/**< global position subscription */
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};		/**< notification of manual control updates */
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		/**< notification of parameter updates */
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};		/**< vehicle rates setpoint */
	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};
	//JUAN
	uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _juan_attitude_variables_sub{ORB_ID(juan_attitude_variables)}; //JUAN
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};		/**< local position subscription */

	uORB::SubscriptionData<airspeed_s> _airspeed_sub{ORB_ID(airspeed)};

	uORB::Publication<actuator_controls_s>		_actuators_2_pub{ORB_ID(actuator_controls_2)};		/**< actuator control group 1 setpoint (Airframe) */
	uORB::Publication<vehicle_rates_setpoint_s>	_rate_sp_pub{ORB_ID(vehicle_rates_setpoint)};		/**< rate setpoint publication */
	uORB::PublicationMulti<rate_ctrl_status_s>	_rate_ctrl_status_pub{ORB_ID(rate_ctrl_status)};	/**< rate controller status publication */
	// JUAN
	uORB::Publication<vehicle_local_position_setpoint_s>	_local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<juan_attitude_variables_s>	_juan_attitude_variables_pub{ORB_ID(juan_attitude_variables)}; //JUAN

	orb_id_t	_attitude_setpoint_id{nullptr};
	orb_advert_t	_attitude_sp_pub{nullptr};	/**< attitude setpoint point */

	orb_id_t	_actuators_id{nullptr};		/**< pointer to correct actuator controls0 uORB metadata structure */
	orb_advert_t	_actuators_0_pub{nullptr};	/**< actuator control group 0 setpoint */

	actuator_controls_s			_actuators {};		/**< actuator control inputs */
	actuator_controls_s			_actuators_airframe {};	/**< actuator control inputs */
	manual_control_setpoint_s		_manual {};		/**< r/c channel data */
	vehicle_attitude_s			_att {};		/**< vehicle attitude setpoint */
	vehicle_attitude_setpoint_s		_att_sp {};		/**< vehicle attitude setpoint */
	vehicle_control_mode_s			_vcontrol_mode {};	/**< vehicle control mode */
	vehicle_global_position_s		_global_pos {};		/**< global position */
	vehicle_rates_setpoint_s		_rates_sp {};		/* attitude rates setpoint */
	vehicle_status_s			_vehicle_status {};	/**< vehicle status */
	//JUAN
	vehicle_local_position_setpoint_s _local_pos_sp{}; //local position setpoint
	juan_attitude_variables_s _juan_att_var{}; // JUAN custom attitude control variables
	vehicle_local_position_s		_local_pos {};		/**< local position */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

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
	int _JUAN_flight_mode{0};
	matrix::Vector3f _omega_reference_body;
	float _throttle_out;
	matrix::Dcmf C_ri;
	float _previous_rpm{900.0f};
	float _advance_ratio{0.5f};
	matrix::Vector3f _alpha_reference_body;
	float _global_jar;
	float att_err_modifier{1.0f};
	float fv1{0.0f};
	float fv2{0.0f};
	float fv3{0.0f};

	int enter_acro{0};
	int flick_counter{1};
	float x_rate_body;
	float y_rate_body;
	float z_rate_body;

	float ddw_x_body{0.0f};
	float ddw_y_body{0.0f};
	float ddw_z_body{0.0f};
	float mc_alpha{0.0f};
	float d_mc_alpha{0.0f};
	float dd_mc_alpha{0.0f};



	float _read_roll_stick;
	float _read_pitch_stick;
	float _read_thrust_stick;
	float _read_yaw_stick;

	float _pos_x_est;
	float _pos_y_est;
	float _pos_z_est;

	float _vel_x_est;
	float _vel_y_est;
	float _vel_z_est;

	int _attitude_maneuver{0};
	int _position_maneuver{0};

	float _manual_yaw;
	float _manual_roll;
	float _manual_pitch;
	float _manual_fourth;
	float _heading_rate_coordinated;

	int _thust_manual_flag{0};
	int _position_control_flag{0};
	int _velocity_control_flag{0};

	float _thrust_hard_value;
	int _program_counter{0};

	float _tau_1;
	float _tau_2;
	float _tau_3;

	float _AilDef;
	float _ElevDef;
	float _RudDef;

	float _sigma{0.0f};
	float _sigma_0{0.0f};
	float _d_sigma;


	float _Tv1;
	float _Tv2;
	float _Tv3;

	float _heading_path;

	int _path_following_flag{0};

	float _Vel_PF;
	float _a_PF;
	float _b_PF;

	float _Vel_PF_prev;
	float _a_PF_prev;
	float _b_PF_prev;



	float _err_vel_x_int{0.0f};
	float _err_vel_y_int{0.0f};
	float _err_vel_z_int{0.0f};

	float _roll_com;

	float _probe_var;

	float _lateral_error_roll;

	float _error_onpath;
	float _error_crosstrack;
	float _error_altitude;

	int _control_mode_change_direction{0};

	float _pos_x_prev{0.0f};
	float _pos_y_prev{0.0f};

	float barrel_roll{0.0f};

	float _slips_estimate{0.0f};

	struct {
		float p_tc;
		float p_p;
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
		float roll_to_yaw_ff;
		float y_rmax;

		bool w_en;
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
		float dtrim_roll_vmin;
		float dtrim_pitch_vmin;
		float dtrim_yaw_vmin;
		float dtrim_roll_vmax;
		float dtrim_pitch_vmax;
		float dtrim_yaw_vmax;
		float dtrim_roll_flaps;
		float dtrim_pitch_flaps;
		float rollsp_offset_deg;		/**< Roll Setpoint Offset in deg */
		float pitchsp_offset_deg;		/**< Pitch Setpoint Offset in deg */
		float rollsp_offset_rad;		/**< Roll Setpoint Offset in rad */
		float pitchsp_offset_rad;		/**< Pitch Setpoint Offset in rad */
		float man_roll_max;			/**< Max Roll in rad */
		float man_pitch_max;			/**< Max Pitch in rad */
		float man_roll_scale;			/**< scale factor applied to roll actuator control in pure manual mode */
		float man_pitch_scale;			/**< scale factor applied to pitch actuator control in pure manual mode */
		float man_yaw_scale; 			/**< scale factor applied to yaw actuator control in pure manual mode */

		float acro_max_x_rate_rad;
		float acro_max_y_rate_rad;
		float acro_max_z_rate_rad;

		float flaps_scale;			/**< Scale factor for flaps */
		float flaps_takeoff_scale;		/**< Scale factor for flaps on take-off */
		float flaps_land_scale;			/**< Scale factor for flaps on landing */
		float flaperon_scale;			/**< Scale factor for flaperons */

		float rattitude_thres;

		int32_t bat_scale_en;			/**< Battery scaling enabled */
		bool airspeed_disabled;







	} _parameters{};			/**< local copies of interesting parameters */

	struct {

		param_t p_tc;
		param_t p_p;
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
		param_t roll_to_yaw_ff;
		param_t y_rmax;

		param_t w_en;
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
		param_t dtrim_roll_vmin;
		param_t dtrim_pitch_vmin;
		param_t dtrim_yaw_vmin;
		param_t dtrim_roll_vmax;
		param_t dtrim_pitch_vmax;
		param_t dtrim_yaw_vmax;
		param_t dtrim_roll_flaps;
		param_t dtrim_pitch_flaps;
		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_roll_scale;
		param_t man_pitch_scale;
		param_t man_yaw_scale;

		param_t acro_max_x_rate;
		param_t acro_max_y_rate;
		param_t acro_max_z_rate;

		param_t flaps_scale;
		param_t flaps_takeoff_scale;
		param_t flaps_land_scale;
		param_t flaperon_scale;

		param_t rattitude_thres;

		param_t bat_scale_en;
		param_t airspeed_mode;

	} _parameter_handles{};		/**< handles for interesting parameters */

	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;
	ECL_WheelController			_wheel_ctrl;

	void control_flaps(const float dt);

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	void		vehicle_control_mode_poll();
	void		vehicle_manual_poll();
	void		vehicle_attitude_setpoint_poll();
	void		vehicle_rates_setpoint_poll();
	void		vehicle_status_poll();
	void		vehicle_land_detected_poll();

	float 		get_airspeed_and_update_scaling();

	//JUAN additional functions
	void    JUAN_position_control();
	void    JUAN_reference_generator();
	float   saturate(float value, float min, float max);
	void    JUAN_singularity_management(float xy_speed, float angle_vect);
	void    JUAN_enter_acro_manager();
	void    JUAN_stabilized_reset();
	void    JUAN_vehicle_states_reader();
	void    JUAN_reference_attitude_generator();

	void    JUAN_straight_line_time(float _Vel, float _Xi, float _Ga);
	void    JUAN_control_allocation();
	void    JUAN_mission_planner();
	void    JUAN_logger();
	void    JUAN_attitude_control(int _innovation_option);
	void    JUAN_pose_initialize();
	void    Unit_Speed_line(float Xi, float Ga);
	void    Unit_Speed_helix(float dir, float r, float c, float Xi);
	void    JUAN_Path_F_Helix_Lines(float Vel, float a, float b);
	void    JUAN_Add_Roll();
	void    JUAN_helix_time(float _Vel, float _dir, float r, float c, float Xi);
	void    JUAN_provisional_path_following();
	void    JUAN_manual_PF_manager();

};
