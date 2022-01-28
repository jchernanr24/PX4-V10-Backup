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

#include "FixedwingAttitudeControl.hpp"

#include <vtol_att_control/vtol_type.h>

#define PI_f  3.1415f

using namespace time_literals;

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[]);

FixedwingAttitudeControl::FixedwingAttitudeControl() :
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, "fw_att_control: cycle"))
{
	// check if VTOL first
	vehicle_status_poll();

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
	_parameter_handles.roll_to_yaw_ff = param_find("FW_RLL_TO_YAW_FF");

	_parameter_handles.w_en = param_find("FW_W_EN");
	_parameter_handles.w_p = param_find("FW_WR_P");
	_parameter_handles.w_i = param_find("FW_WR_I");
	_parameter_handles.w_ff = param_find("FW_WR_FF");
	_parameter_handles.w_integrator_max = param_find("FW_WR_IMAX");
	_parameter_handles.w_rmax = param_find("FW_W_RMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");
	_parameter_handles.dtrim_roll_vmin = param_find("FW_DTRIM_R_VMIN");
	_parameter_handles.dtrim_pitch_vmin = param_find("FW_DTRIM_P_VMIN");
	_parameter_handles.dtrim_yaw_vmin = param_find("FW_DTRIM_Y_VMIN");
	_parameter_handles.dtrim_roll_vmax = param_find("FW_DTRIM_R_VMAX");
	_parameter_handles.dtrim_pitch_vmax = param_find("FW_DTRIM_P_VMAX");
	_parameter_handles.dtrim_yaw_vmax = param_find("FW_DTRIM_Y_VMAX");
	_parameter_handles.dtrim_roll_flaps = param_find("FW_DTRIM_R_FLPS");
	_parameter_handles.dtrim_pitch_flaps = param_find("FW_DTRIM_P_FLPS");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.man_roll_max = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max = param_find("FW_MAN_P_MAX");
	_parameter_handles.man_roll_scale = param_find("FW_MAN_R_SC");
	_parameter_handles.man_pitch_scale = param_find("FW_MAN_P_SC");
	_parameter_handles.man_yaw_scale = param_find("FW_MAN_Y_SC");

	_parameter_handles.acro_max_x_rate = param_find("FW_ACRO_X_MAX");
	_parameter_handles.acro_max_y_rate = param_find("FW_ACRO_Y_MAX");
	_parameter_handles.acro_max_z_rate = param_find("FW_ACRO_Z_MAX");

	_parameter_handles.flaps_scale = param_find("FW_FLAPS_SCL");
	_parameter_handles.flaps_takeoff_scale = param_find("FW_FLAPS_TO_SCL");
	_parameter_handles.flaps_land_scale = param_find("FW_FLAPS_LND_SCL");
	_parameter_handles.flaperon_scale = param_find("FW_FLAPERON_SCL");

	_parameter_handles.rattitude_thres = param_find("FW_RATT_TH");

	_parameter_handles.bat_scale_en = param_find("FW_BAT_SCALE_EN");
	_parameter_handles.airspeed_mode = param_find("FW_ARSP_MODE");

	/* fetch initial parameter values */
	parameters_update();

	// set initial maximum body rate setpoints
	_roll_ctrl.set_max_rate(_parameters.acro_max_x_rate_rad);
	_pitch_ctrl.set_max_rate_pos(_parameters.acro_max_y_rate_rad);
	_pitch_ctrl.set_max_rate_neg(_parameters.acro_max_y_rate_rad);
	_yaw_ctrl.set_max_rate(_parameters.acro_max_z_rate_rad);
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingAttitudeControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("vehicle attitude callback registration failed!");
		return false;
	}

	return true;
}

int
FixedwingAttitudeControl::parameters_update()
{
	int32_t tmp = 0;
	param_get(_parameter_handles.p_tc, &(_parameters.p_tc));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
	param_get(_parameter_handles.p_rmax_pos, &(_parameters.p_rmax_pos));
	param_get(_parameter_handles.p_rmax_neg, &(_parameters.p_rmax_neg));
	param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));

	param_get(_parameter_handles.r_tc, &(_parameters.r_tc));
	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_ff, &(_parameters.r_ff));

	param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));
	param_get(_parameter_handles.y_ff, &(_parameters.y_ff));
	param_get(_parameter_handles.y_integrator_max, &(_parameters.y_integrator_max));
	param_get(_parameter_handles.y_rmax, &(_parameters.y_rmax));
	param_get(_parameter_handles.roll_to_yaw_ff, &(_parameters.roll_to_yaw_ff));

	param_get(_parameter_handles.w_en, &tmp);
	_parameters.w_en = (tmp == 1);

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
	param_get(_parameter_handles.dtrim_roll_vmin, &(_parameters.dtrim_roll_vmin));
	param_get(_parameter_handles.dtrim_roll_vmax, &(_parameters.dtrim_roll_vmax));
	param_get(_parameter_handles.dtrim_pitch_vmin, &(_parameters.dtrim_pitch_vmin));
	param_get(_parameter_handles.dtrim_pitch_vmax, &(_parameters.dtrim_pitch_vmax));
	param_get(_parameter_handles.dtrim_yaw_vmin, &(_parameters.dtrim_yaw_vmin));
	param_get(_parameter_handles.dtrim_yaw_vmax, &(_parameters.dtrim_yaw_vmax));

	param_get(_parameter_handles.dtrim_roll_flaps, &(_parameters.dtrim_roll_flaps));
	param_get(_parameter_handles.dtrim_pitch_flaps, &(_parameters.dtrim_pitch_flaps));

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

	param_get(_parameter_handles.acro_max_x_rate, &(_parameters.acro_max_x_rate_rad));
	param_get(_parameter_handles.acro_max_y_rate, &(_parameters.acro_max_y_rate_rad));
	param_get(_parameter_handles.acro_max_z_rate, &(_parameters.acro_max_z_rate_rad));
	_parameters.acro_max_x_rate_rad = math::radians(_parameters.acro_max_x_rate_rad);
	_parameters.acro_max_y_rate_rad = math::radians(_parameters.acro_max_y_rate_rad);
	_parameters.acro_max_z_rate_rad = math::radians(_parameters.acro_max_z_rate_rad);

	param_get(_parameter_handles.flaps_scale, &_parameters.flaps_scale);
	param_get(_parameter_handles.flaps_takeoff_scale, &_parameters.flaps_takeoff_scale);
	param_get(_parameter_handles.flaps_land_scale, &_parameters.flaps_land_scale);
	param_get(_parameter_handles.flaperon_scale, &_parameters.flaperon_scale);

	param_get(_parameter_handles.rattitude_thres, &_parameters.rattitude_thres);

	param_get(_parameter_handles.bat_scale_en, &_parameters.bat_scale_en);

	param_get(_parameter_handles.airspeed_mode, &tmp);
	_parameters.airspeed_disabled = (tmp == 1);

	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.p_tc);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.r_tc);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);

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
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	if (_vehicle_status.is_vtol) {
		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && !_vehicle_status.in_transition_mode;
		const bool is_tailsitter_transition = _vehicle_status.in_transition_mode && _is_tailsitter;

		if (is_hovering || is_tailsitter_transition) {
			_vcontrol_mode.flag_control_attitude_enabled = false;
			_vcontrol_mode.flag_control_manual_enabled = false;
		}
	}
}

void
FixedwingAttitudeControl::vehicle_manual_poll()
{
	const bool is_tailsitter_transition = _is_tailsitter && _vehicle_status.in_transition_mode;
	const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	if (_vcontrol_mode.flag_control_manual_enabled && (!is_tailsitter_transition || is_fixed_wing)) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the _actuators with valid values
		if (_manual_sub.copy(&_manual)) {

			// Check if we are in rattitude mode and the pilot is above the threshold on pitch
			if (_vcontrol_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual.y) > _parameters.rattitude_thres || fabsf(_manual.x) > _parameters.rattitude_thres) {
					_vcontrol_mode.flag_control_attitude_enabled = false;
				}
			}

			if (!_vcontrol_mode.flag_control_climb_rate_enabled &&
			    !_vcontrol_mode.flag_control_offboard_enabled) {

				if (_vcontrol_mode.flag_control_attitude_enabled) {
					// STABILIZED mode generate the attitude setpoint from manual user inputs
					_att_sp.timestamp = hrt_absolute_time();
					_att_sp.roll_body = _manual.y * _parameters.man_roll_max + _parameters.rollsp_offset_rad;
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, -_parameters.man_roll_max, _parameters.man_roll_max);
					_att_sp.pitch_body = -_manual.x * _parameters.man_pitch_max + _parameters.pitchsp_offset_rad;
					_att_sp.pitch_body = math::constrain(_att_sp.pitch_body, -_parameters.man_pitch_max, _parameters.man_pitch_max);
					_att_sp.yaw_body = 0.0f;
					_att_sp.thrust_body[0] = _manual.z;


					Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					q.copyTo(_att_sp.q_d);
					_att_sp.q_d_valid = true;

					if (_attitude_sp_pub != nullptr) {
						/* publish the attitude rates setpoint */
						orb_publish(_attitude_setpoint_id, _attitude_sp_pub, &_att_sp);

					} else if (_attitude_setpoint_id) {
						/* advertise the attitude rates setpoint */
						_attitude_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
					}

				} else if (_vcontrol_mode.flag_control_rates_enabled &&
					   !_vcontrol_mode.flag_control_attitude_enabled) {

					// RATE mode we need to generate the rate setpoint from manual user inputs
					_rates_sp.timestamp = hrt_absolute_time();
					_rates_sp.roll = _manual.y * _parameters.acro_max_x_rate_rad;
					_rates_sp.pitch = -_manual.x * _parameters.acro_max_y_rate_rad;
					_rates_sp.yaw = _manual.r * _parameters.acro_max_z_rate_rad;
					_rates_sp.thrust_body[0] = _manual.z;

					_rate_sp_pub.publish(_rates_sp);




				} else {
					/* manual/direct control */
					_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y * _parameters.man_roll_scale + _parameters.trim_roll;
					_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x * _parameters.man_pitch_scale +
							_parameters.trim_pitch;
					_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _parameters.man_yaw_scale + _parameters.trim_yaw;
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
				}
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_attitude_setpoint_poll()
{
	if (_att_sp_sub.update(&_att_sp)) {
		_rates_sp.thrust_body[0] = _att_sp.thrust_body[0];
		_rates_sp.thrust_body[1] = _att_sp.thrust_body[1];
		_rates_sp.thrust_body[2] = _att_sp.thrust_body[2];
	}
}

void
FixedwingAttitudeControl::vehicle_rates_setpoint_poll()
{
	if (_rates_sp_sub.update(&_rates_sp)) {
		if (_is_tailsitter) {
			float tmp = _rates_sp.roll;
			_rates_sp.roll = -_rates_sp.yaw;
			_rates_sp.yaw = tmp;
		}
	}
}

void
FixedwingAttitudeControl::vehicle_status_poll()
{
	if (_vehicle_status_sub.update(&_vehicle_status)) {
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_actuators_id) {
			if (_vehicle_status.is_vtol) {
				_actuators_id = ORB_ID(actuator_controls_virtual_fw);
				_attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

				int32_t vt_type = -1;

				if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
					_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
				}

			} else {
				_actuators_id = ORB_ID(actuator_controls_0);
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_land_detected_poll()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

float FixedwingAttitudeControl::get_airspeed_and_update_scaling()
{
	_airspeed_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_sub.get().indicated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_sub.get().timestamp) < 1_s)
				    && !_vehicle_status.aspd_use_inhibit;

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _parameters.airspeed_trim;

	if (!_parameters.airspeed_disabled && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_sub.get().indicated_airspeed_m_s);

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the minimum airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _parameters.airspeed_min;
		}
	}

	/*
	 * For scaling our actuators using anything less than the min (close to stall)
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and its the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	const float airspeed_constrained = math::constrain(airspeed, _parameters.airspeed_min, _parameters.airspeed_max);
	_airspeed_scaling = _parameters.airspeed_trim / airspeed_constrained;

	return airspeed;
}

void FixedwingAttitudeControl::Run()
{
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_att_sub.update(&_att)) {

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			parameters_update();
		}

		/* only run controller if attitude changed */
		static uint64_t last_run = 0;
		float deltaT = math::constrain((hrt_elapsed_time(&last_run) / 1e6f), 0.01f, 0.1f);
		last_run = hrt_absolute_time();

		/* get current rotation matrix and euler angles from control state quaternions */
		matrix::Dcmf R = matrix::Quatf(_att.q);

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_rates_sub.copy(&angular_velocity);
		float rollspeed = angular_velocity.xyz[0];
		float pitchspeed = angular_velocity.xyz[1];
		float yawspeed = angular_velocity.xyz[2];

		if (_is_tailsitter) {
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
			matrix::Dcmf R_adapted = R;		//modified rotation matrix

			/* move z to x */
			R_adapted(0, 0) = R(0, 2);
			R_adapted(1, 0) = R(1, 2);
			R_adapted(2, 0) = R(2, 2);

			/* move x to z */
			R_adapted(0, 2) = R(0, 0);
			R_adapted(1, 2) = R(1, 0);
			R_adapted(2, 2) = R(2, 0);

			/* change direction of pitch (convert to right handed system) */
			R_adapted(0, 0) = -R_adapted(0, 0);
			R_adapted(1, 0) = -R_adapted(1, 0);
			R_adapted(2, 0) = -R_adapted(2, 0);

			/* fill in new attitude data */
			R = R_adapted;

			/* lastly, roll- and yawspeed have to be swaped */
			float helper = rollspeed;
			rollspeed = -yawspeed;
			yawspeed = helper;
		}

		const matrix::Eulerf euler_angles(R);

		vehicle_attitude_setpoint_poll();
		vehicle_status_poll(); // this poll has to be before the control_mode_poll, otherwise rate sp are not published during whole transition
		vehicle_control_mode_poll();
		vehicle_manual_poll();
		_global_pos_sub.update(&_global_pos);
		_local_pos_sub.update(&_local_pos);
		vehicle_land_detected_poll();






		// the position controller will not emit attitude setpoints in some modes
		// we need to make sure that this flag is reset
		_att_sp.fw_control_yaw = _att_sp.fw_control_yaw && _vcontrol_mode.flag_control_auto_enabled;

		/* lock integrator until control is started */
		// bool lock_integrator = !_vcontrol_mode.flag_control_rates_enabled
		// 		       || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && ! _vehicle_status.in_transition_mode);


		/* JUAN modified integrator lock: will lock while in acro, unlock in stabilized*/
		bool lock_integrator = !(_vcontrol_mode.flag_control_rates_enabled && _vcontrol_mode.flag_control_attitude_enabled)
				       || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && ! _vehicle_status.in_transition_mode);



		/* Simple handling of failsafe: deploy parachute if failsafe is on */
		if (_vcontrol_mode.flag_control_termination_enabled) {
			_actuators_airframe.control[7] = 1.0f;

		} else {
			_actuators_airframe.control[7] = 0.0f;
		}

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}

		control_flaps(deltaT);

		/* decide if in stabilized or full manual control */
		if (_vcontrol_mode.flag_control_rates_enabled) {

			const float airspeed = get_airspeed_and_update_scaling();

			/* Use min airspeed to calculate ground speed scaling region.
			 * Don't scale below gspd_scaling_trim
			 */
			float groundspeed = sqrtf(_global_pos.vel_n * _global_pos.vel_n +
						  _global_pos.vel_e * _global_pos.vel_e);
			float gspd_scaling_trim = (_parameters.airspeed_min * 0.6f);
			float groundspeed_scaler = gspd_scaling_trim / ((groundspeed < gspd_scaling_trim) ? gspd_scaling_trim : groundspeed);

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

			/* Reset integrators if the aircraft is on ground
			 * or a multicopter (but not transitioning VTOL)
			 */
			if (_landed
			    || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				&& !_vehicle_status.in_transition_mode)) {

				_roll_ctrl.reset_integrator();
				_pitch_ctrl.reset_integrator();
				_yaw_ctrl.reset_integrator();
				_wheel_ctrl.reset_integrator();
			}

			/* Prepare data for attitude controllers */
			struct ECL_ControlData control_input = {};
			control_input.roll = euler_angles.phi();
			control_input.pitch = euler_angles.theta();
			control_input.yaw = euler_angles.psi();
			control_input.body_x_rate = rollspeed;
			control_input.body_y_rate = pitchspeed;
			control_input.body_z_rate = yawspeed;
			control_input.roll_setpoint = _att_sp.roll_body;
			control_input.pitch_setpoint = _att_sp.pitch_body;
			control_input.yaw_setpoint = _att_sp.yaw_body;
			control_input.airspeed_min = _parameters.airspeed_min;
			control_input.airspeed_max = _parameters.airspeed_max;
			control_input.airspeed = airspeed;
			control_input.scaler = _airspeed_scaling;
			control_input.lock_integrator = lock_integrator;
			control_input.groundspeed = groundspeed;
			control_input.groundspeed_scaler = groundspeed_scaler;

			/* reset body angular rate limits on mode change */
			if ((_vcontrol_mode.flag_control_attitude_enabled != _flag_control_attitude_enabled_last) || params_updated) {
				if (_vcontrol_mode.flag_control_attitude_enabled
				    || _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));
					_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
					_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));
					_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));

				} else {
					_roll_ctrl.set_max_rate(_parameters.acro_max_x_rate_rad);
					_pitch_ctrl.set_max_rate_pos(_parameters.acro_max_y_rate_rad);
					_pitch_ctrl.set_max_rate_neg(_parameters.acro_max_y_rate_rad);
					_yaw_ctrl.set_max_rate(_parameters.acro_max_z_rate_rad);
				}
			}

			_flag_control_attitude_enabled_last = _vcontrol_mode.flag_control_attitude_enabled;

			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			float trim_roll = _parameters.trim_roll;
			float trim_pitch = _parameters.trim_pitch;
			float trim_yaw = _parameters.trim_yaw;

			if (airspeed < _parameters.airspeed_trim) {
				trim_roll += math::gradual(airspeed, _parameters.airspeed_min, _parameters.airspeed_trim, _parameters.dtrim_roll_vmin,
							   0.0f);
				trim_pitch += math::gradual(airspeed, _parameters.airspeed_min, _parameters.airspeed_trim, _parameters.dtrim_pitch_vmin,
							    0.0f);
				trim_yaw += math::gradual(airspeed, _parameters.airspeed_min, _parameters.airspeed_trim, _parameters.dtrim_yaw_vmin,
							  0.0f);

			} else {
				trim_roll += math::gradual(airspeed, _parameters.airspeed_trim, _parameters.airspeed_max, 0.0f,
							   _parameters.dtrim_roll_vmax);
				trim_pitch += math::gradual(airspeed, _parameters.airspeed_trim, _parameters.airspeed_max, 0.0f,
							    _parameters.dtrim_pitch_vmax);
				trim_yaw += math::gradual(airspeed, _parameters.airspeed_trim, _parameters.airspeed_max, 0.0f,
							  _parameters.dtrim_yaw_vmax);
			}

			/* add trim increment if flaps are deployed  */
			trim_roll += _flaps_applied * _parameters.dtrim_roll_flaps;
			trim_pitch += _flaps_applied * _parameters.dtrim_pitch_flaps;

			/* Run attitude controllers */
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				if (PX4_ISFINITE(_att_sp.roll_body) && PX4_ISFINITE(_att_sp.pitch_body)) {
					_roll_ctrl.control_attitude(control_input);
					_pitch_ctrl.control_attitude(control_input);
					_yaw_ctrl.control_attitude(control_input); //runs last, because is depending on output of roll and pitch attitude
					_wheel_ctrl.control_attitude(control_input);

					/* Update input data for rate controllers */
					control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
					control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
					control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

					/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
					float roll_u = _roll_ctrl.control_euler_rate(control_input);
					_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;

					if (!PX4_ISFINITE(roll_u)) {
						_roll_ctrl.reset_integrator();
					}

					float pitch_u = _pitch_ctrl.control_euler_rate(control_input);
					_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;

					if (!PX4_ISFINITE(pitch_u)) {
						_pitch_ctrl.reset_integrator();
					}

					float yaw_u = 0.0f;

					if (_parameters.w_en && _att_sp.fw_control_yaw) {
						yaw_u = _wheel_ctrl.control_bodyrate(control_input);

					} else {
						yaw_u = _yaw_ctrl.control_euler_rate(control_input);
					}

					_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

					/* add in manual rudder control in manual modes */
					if (_vcontrol_mode.flag_control_manual_enabled) {
						_actuators.control[actuator_controls_s::INDEX_YAW] += _manual.r;
					}

					if (!PX4_ISFINITE(yaw_u)) {
						_yaw_ctrl.reset_integrator();
						_wheel_ctrl.reset_integrator();
					}

					/* throttle passed through if it is finite and if no engine failure was detected */
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(_att_sp.thrust_body[0])
							&& !_vehicle_status.engine_failure) ? _att_sp.thrust_body[0] : 0.0f;

					/* scale effort by battery status */
					if (_parameters.bat_scale_en &&
					    _actuators.control[actuator_controls_s::INDEX_THROTTLE] > 0.1f) {

						if (_battery_status_sub.updated()) {
							battery_status_s battery_status{};

							if (_battery_status_sub.copy(&battery_status)) {
								if (battery_status.scale > 0.0f) {
									_battery_scale = battery_status.scale;
								}
							}
						}

						_actuators.control[actuator_controls_s::INDEX_THROTTLE] *= _battery_scale;
					}
				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				_rates_sp.roll = _roll_ctrl.get_desired_bodyrate();
				_rates_sp.pitch = _pitch_ctrl.get_desired_bodyrate();
				_rates_sp.yaw = _yaw_ctrl.get_desired_bodyrate();

				_rates_sp.timestamp = hrt_absolute_time();

				_rate_sp_pub.publish(_rates_sp);

				// JUAN: Preparing variables for custom mode transition
				JUAN_stabilized_reset();
				_previous_yaw = euler_angles.psi();
				_initial_heading = _previous_yaw;
				_heading_path = _initial_heading;
				_previous_time = hrt_absolute_time()/1e6;
				_time_elapsed = 0.0f;
				_e_int_1 = 0.0f;
				_e_int_2 = 0.0f;
				_e_int_3 = 0.0f;



				_control_operation_mode = 0;


				_yaw_test_profile = _previous_yaw;
				_pitch_test_profile = 0.0f;
				_roll_test_profile = 0.0f;

				_local_pos_sub.update(&_local_pos);
				float _added_initial_distance = 10.0f;

				_pos_x_ref = _local_pos.x+_added_initial_distance*cosf(_initial_heading);
				_pos_y_ref = _local_pos.y+_added_initial_distance*sinf(_initial_heading);
				_pos_z_ref = _local_pos.z;

				_pos_x_initial = _pos_x_ref;
				_pos_y_initial = _pos_y_ref;
				_pos_z_initial = _pos_z_ref;


				_vel_x_ref = _local_pos.vx;
				_vel_y_ref = _local_pos.vy;
				_vel_z_ref = _local_pos.vz;

				_error_heading_int = 0.0f;

				_JUAN_flight_mode = 0;

				_previous_rpm = 900.0f;


			} else {
				vehicle_rates_setpoint_poll();

				_roll_ctrl.set_bodyrate_setpoint(_rates_sp.roll);
				_yaw_ctrl.set_bodyrate_setpoint(_rates_sp.yaw);
				_pitch_ctrl.set_bodyrate_setpoint(_rates_sp.pitch);

				// float roll_u = _roll_ctrl.control_bodyrate(control_input);
				// _actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;
				//
				// float pitch_u = _pitch_ctrl.control_bodyrate(control_input);
				// _actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;
				//
				// float yaw_u = _yaw_ctrl.control_bodyrate(control_input);
				// _actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;
				//
				// _actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ?
				// 		_rates_sp.thrust_body[0] : 0.0f;


				// /* JUAN attitude control starts here */

					JUAN_enter_acro_manager();     //Check times flicked and time management
					JUAN_vehicle_states_reader();  //Read states and stick inputs

					/*...........Explanation of Controller workflow .................*/
					// The controller works on 4 main layers: Mission management,
					// pos. reference generation, position control, and attitude control.


					// First, the state machine, or commander, determines the order in
					// which every flight mode is commanded.
					// The mission is built by the user, can be full auto, semiauto,
					// or a combination. State transitions are either time based, pose
					// based (i.e. altitude) or pilot input based.

					// The next task is position reference generation. If in tracking mode,
					// the commander invokes the reference generator which has a
					// predefined time function, or creates one through pilot input.
					// If in path following, the path generator gets called. This computes
					// reference velocity for the tracking controller to use, either through
					// a predescribed geometry, or stitched together through user input.

					// The position controller is then invoked, using the reference
					// previously generated. The tracking controller generates thrust and
					// attitude reference.

					// If operating in attitude mode, the position controller is not used.
					// Attitude reference is generated through the att. ref. generator.
					// and thrust is hardcoded or passed by the user.
					// Regardless of mode, the attitude controller is always called. This
					// generates the final desired moments.

					// Next, the control allocation function is called to determine
					// control surface deflections and motor rpm.

					// Finally, the logging function is called to register relevant
					// variables. Some variables are only logged in specific modes.
					/*..........Explanation end .....................................*/



					JUAN_mission_planner();

					if (_position_control_flag == 0) {
						JUAN_reference_attitude_generator();
					}
					else{
						 // if (_path_following_flag == 0)
						 // {
						 // 	 JUAN_reference_generator();
						 // }
						 // else{
							//  JUAN_Path_F_Helix_Lines(_Vel_PF, _a_PF, _b_PF);
						 // }
						 JUAN_position_control();
						 C_ri = C_ri_pos.transpose();
						 matrix::Vector3f _omega_ref_temp(0.0f, 0.0f, 0.0f); // change for command filter
						 _omega_reference_body = _omega_ref_temp;
					}


					/*... DCMs for control system ......................................*/

					// if (_JUAN_flight_mode < 1)
					// {
					// 	// _attitude_maneuver = 0;
					// 	// Generate attitude references for pure attitude control.
					// 	// Includes manual attitude modes.
					// 	JUAN_reference_attitude_generator();
					// /*..................................................................*/
					// 	// _throttle_out = 1;
					// 	// _position_control_flag = 0;
					// 	// _thust_manual_flag = 1;
				 //
				 //
				 // }
				 // else
				 // {
					//  // Call JUAN Maneuver generator. This assigns a position setpoint.
				 // 	 // NOTE!!!!!!!!!!! Check Qground disarm parameters!!!!!!
				 //
					//  _position_maneuver = 1;
					//  JUAN_reference_generator();
					//  JUAN_position_control();
					//  C_ri = C_ri_pos.transpose();
					//  matrix::Vector3f _omega_ref_temp(0.0f, 0.0f, 0.0f); // change for command filter
					//  _omega_reference_body = _omega_ref_temp;
					//  _position_control_flag = 1;
					//  _thust_manual_flag = 0;
				 // }

				 	JUAN_attitude_control(_attitude_error_innovation);
					/* .........................Control allocation......................*/
					JUAN_control_allocation();
					JUAN_logger();

				/*............... JUAN attitude control ends here.................... */

			}

			rate_ctrl_status_s rate_ctrl_status;
			rate_ctrl_status.timestamp = hrt_absolute_time();
			rate_ctrl_status.rollspeed_integ = _roll_ctrl.get_integrator();
			rate_ctrl_status.pitchspeed_integ = _pitch_ctrl.get_integrator();
			rate_ctrl_status.yawspeed_integ = _yaw_ctrl.get_integrator();
			rate_ctrl_status.additional_integ1 = _wheel_ctrl.get_integrator();

			_rate_ctrl_status_pub.publish(rate_ctrl_status);
		}

		// Add feed-forward from roll control output to yaw control output
		// This can be used to counteract the adverse yaw effect when rolling the plane
		_actuators.control[actuator_controls_s::INDEX_YAW] += _parameters.roll_to_yaw_ff * math::constrain(
					_actuators.control[actuator_controls_s::INDEX_ROLL], -1.0f, 1.0f);

		_actuators.control[actuator_controls_s::INDEX_FLAPS] = _flaps_applied;
		_actuators.control[5] = _manual.aux1;
		_actuators.control[actuator_controls_s::INDEX_AIRBRAKES] = _flaperons_applied;
		// FIXME: this should use _vcontrol_mode.landing_gear_pos in the future
		_actuators.control[7] = _manual.aux3;

		/* lazily publish the setpoint only once available */
		_actuators.timestamp = hrt_absolute_time();
		_actuators.timestamp_sample = _att.timestamp;
		_actuators_airframe.timestamp = hrt_absolute_time();
		_actuators_airframe.timestamp_sample = _att.timestamp;

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

			_actuators_2_pub.publish(_actuators_airframe);
		}
	}

	perf_end(_loop_perf);
}

void FixedwingAttitudeControl::control_flaps(const float dt)
{
	/* default flaps to center */
	float flap_control = 0.0f;

	/* map flaps by default to manual if valid */
	if (PX4_ISFINITE(_manual.flaps) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_parameters.flaps_scale) > 0.01f) {
		flap_control = 0.5f * (_manual.flaps + 1.0f) * _parameters.flaps_scale;

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_parameters.flaps_scale) > 0.01f) {
		switch (_att_sp.apply_flaps) {
		case vehicle_attitude_setpoint_s::FLAPS_OFF : flap_control = 0.0f; // no flaps
			break;

		case vehicle_attitude_setpoint_s::FLAPS_LAND : flap_control = 1.0f * _parameters.flaps_scale *
					_parameters.flaps_land_scale; // landing flaps
			break;

		case vehicle_attitude_setpoint_s::FLAPS_TAKEOFF : flap_control = 1.0f * _parameters.flaps_scale *
					_parameters.flaps_takeoff_scale; // take-off flaps
			break;
		}
	}

	// move the actual control value continuous with time, full flap travel in 1sec
	if (fabsf(_flaps_applied - flap_control) > 0.01f) {
		_flaps_applied += (_flaps_applied - flap_control) < 0 ? dt : -dt;

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
		flaperon_control = (_att_sp.apply_flaps == vehicle_attitude_setpoint_s::FLAPS_LAND) ? 1.0f *
				   _parameters.flaperon_scale : 0.0f;
	}

	// move the actual control value continuous with time, full flap travel in 1sec
	if (fabsf(_flaperons_applied - flaperon_control) > 0.01f) {
		_flaperons_applied += (_flaperons_applied - flaperon_control) < 0 ? dt : -dt;

	} else {
		_flaperons_applied = flaperon_control;
	}
}

int FixedwingAttitudeControl::task_spawn(int argc, char *argv[])
{
	FixedwingAttitudeControl *instance = new FixedwingAttitudeControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FixedwingAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_NAME("fw_att_control", "controller");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FixedwingAttitudeControl::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	return 0;
}

int fw_att_control_main(int argc, char *argv[])
{
	return FixedwingAttitudeControl::main(argc, argv);
}


void FixedwingAttitudeControl::JUAN_position_control()
{
	// Not variables; Make these constants later
	float _gravity_const = 9.81f;
	float _mass_const = 0.45f;
	// Position Control Gains
	// float KpX = 2.5f*0.75f*0.6f*1.0f*3.0f*0.1f*1.8f;
	// float KpY = 2.5f*0.75f*0.6f*1.0f*3.0f*0.1f*1.8f;
	// float KpZ = 2.5f*0.75f*0.7f*6.0f*0.1f*6.0f;
	// float KdX = 2.5f*0.7f*0.9f*1.0f*0.5f*0.42f;
	// float KdY = 2.5f*0.7f*0.9f*1.0f*0.5f*0.42f;
	// float KdZ = 2.0f*0.7f*0.9f*1.0f*0.5f*0.21f;
	// float KiX = 0.25f*0.0008f;
	// float KiY = 0.25f*0.0008f;
	// float KiZ = 0.25f*0.0004f;


	// Thesis experiments gains!!!!!!!!!!!!!!!!
	// float KpX = 1.6f*0.4f*2.0f*0.243f;
	// float KpY = 1.6f*0.4f*2.0f*0.243f;
	// float KpZ = 1.2f*0.4f*2.3f; //1.89f;
	// float KdX = 1.4f*2.5f*0.1323f;
	// float KdY = 1.4f*2.5f*0.1323f;
	// float KdZ = 1.4f*2.0f*0.06615f;
	// float KiX = 0.2f*0.15f*0.0008f;
	// float KiY = 0.2f*0.15f*0.0008f;
	// float KiZ = 0.2f*0.15f*0.0004f;


	float KpX = 1.08f;
	float KpY = 1.08f;
	float KpZ = 3.6f; //1.89f;
	float KdX = 0.672f;
	float KdY = 0.672f;
	float KdZ = 0.336f;
	float KiX = 0.0008f;
	float KiY = 0.0008f;
	float KiZ = 0.0004f;






	// Assign measured position and velocity
	// _local_pos_sub.update(&_local_pos);
	// float _pos_x_est = _local_pos.x;
	// float _pos_y_est = _local_pos.y;
	// float _pos_z_est = _local_pos.z;
	// float _vel_x_est = _local_pos.vx;
	// float _vel_y_est = _local_pos.vy;
	// float _vel_z_est = _local_pos.vz;


	// Control law


	if (_velocity_control_flag == 0)
	{
			float _error_pos_x = _pos_x_ref-_pos_x_est;
			float _error_pos_y = _pos_y_ref-_pos_y_est;
			float _error_pos_z = _pos_z_ref-_pos_z_est;

			float _error_vel_x = _vel_x_ref-_vel_x_est;
			float _error_vel_y = _vel_y_ref-_vel_y_est;
			float _error_vel_z = _vel_z_ref-_vel_z_est;

			_error_x_int = _error_x_int + _error_pos_x*_delta_time_attitude;
			_error_y_int = _error_y_int + _error_pos_y*_delta_time_attitude;
			_error_z_int = _error_z_int + _error_pos_z*_delta_time_attitude;

			// Nose vector
			float Fv1 = 1.0f*0.8f*0.8f*(1.3f*KdX * _error_vel_x + KpX * _error_pos_x + 0.5f*KiX * _error_x_int);
			float Fv2 = 1.0f*0.8f*0.8f*(1.3f*KdY * _error_vel_y + KpY * _error_pos_y + 0.5f*KiY * _error_y_int);
			float Fv3 = 1.0f*0.8f*0.8f*(1.2f*KdZ * _error_vel_z + 1.2f*KpZ * _error_pos_z + 0.3f*KiZ * _error_z_int) - 0.2f*_gravity_const;

			// float Fv1 = 1.0f*0.8f*0.8f*(0.8*1.3f*KdX * _error_vel_x + 0.65*KpX * _error_pos_x + 0.5f*KiX * _error_x_int);
			// float Fv2 = 1.0f*0.8f*0.8f*(0.8*1.3f*KdY * _error_vel_y + 0.65*KpY * _error_pos_y + 0.5f*KiY * _error_y_int);
			// float Fv3 = 1.0f*0.8f*0.8f*(0.8*1.2f*KdZ * _error_vel_z + 0.65*1.2f*KpZ * _error_pos_z + 0.3f*KiZ * _error_z_int) - 0.2f*_gravity_const;

			float _norm_F = sqrtf(Fv1*Fv1+Fv2*Fv2+Fv3*Fv3);

			fv1 = Fv1/_norm_F;
			fv2 = Fv2/_norm_F;
			fv3 = Fv3/_norm_F;
			// Thrust magnitude (N)
			ThrustN = _mass_const*_norm_F;
  }
	else
	{
		float _error_vel_x = _vel_x_ref-_vel_x_est;
		float _error_vel_y = _vel_y_ref-_vel_y_est;
		float _error_vel_z = _vel_z_ref-_vel_z_est;

		_err_vel_x_int = _err_vel_x_int + _error_vel_x*_delta_time_attitude;
		_err_vel_y_int = _err_vel_y_int + _error_vel_y*_delta_time_attitude;
		_err_vel_z_int = _err_vel_z_int + _error_vel_z*_delta_time_attitude;

		float Fv1 = 1.0f*0.8f*0.8f*(1.3f*KdX * _error_vel_x + KpX * _err_vel_x_int );
		float Fv2 = 1.0f*0.8f*0.8f*(1.3f*KdY * _error_vel_y + KpY * _err_vel_y_int );
		float Fv3 = 1.0f*0.8f*0.8f*(1.2f*KdZ * _error_vel_z + 1.2f*KpZ * _err_vel_z_int) - 0.2f*_gravity_const;
		float _norm_F = sqrtf(Fv1*Fv1+Fv2*Fv2+Fv3*Fv3);
		fv1 = Fv1/_norm_F;
		fv2 = Fv2/_norm_F;
		fv3 = Fv3/_norm_F;
		// Thrust magnitude (N)
		ThrustN = _mass_const*_norm_F;
	}





	// Non-normalized wing vector
	float Wv1 = -fv2;
	float Wv2 = fv1;
	float Wv3 = 0.0f;

	float _vel_xy_ref = sqrtf(_vel_x_ref*_vel_x_ref+_vel_y_ref*_vel_y_ref);
	float _norm_W = sqrtf(Wv1*Wv1+Wv2*Wv2);
	float angle_test = asinf(_norm_W);
	JUAN_singularity_management(_vel_xy_ref,angle_test);
	// _control_operation_mode = 0; //remove

	if (_control_operation_mode < 1)
	{
			float wv1 = Wv1/_norm_W;
			float wv2 = Wv2/_norm_W;
			float wv3 = 0.0f;

			float proj1 = wv3*fv2-wv2*fv3;
			float proj2 = wv1*fv3-wv3*fv1;
			float proj3 = wv2*fv1-wv1*fv2;

			belly_n_old = proj1;
			belly_e_old = proj2;


			JUAN_Add_Roll();
			// _roll_com = 0.0f;

			float m_ba11 = fv1;
			float m_ba12 = fv2;
			float m_ba13 = fv3;

			float m_ba21 =  wv1*cosf(_roll_com) + proj1*sinf(_roll_com);
			float m_ba22 =  wv2*cosf(_roll_com) + proj2*sinf(_roll_com);
			float m_ba23 =  wv3*cosf(_roll_com) + proj3*sinf(_roll_com);

			float m_ba31 =  -wv1*sinf(_roll_com) + proj1*cosf(_roll_com);
			float m_ba32 =  -wv2*sinf(_roll_com) + proj2*cosf(_roll_com);
			float m_ba33 =  -wv3*sinf(_roll_com) + proj3*cosf(_roll_com);

				float bldr_array_cri[9] = {m_ba11,m_ba21,m_ba31,m_ba12,m_ba22,m_ba32,m_ba13,m_ba23,m_ba33};
				matrix::SquareMatrix<float, 3> Bldr_Matrix_cri(bldr_array_cri);
				matrix::Dcmf C_ref_steady (Bldr_Matrix_cri);
				C_ri_pos = C_ref_steady;
	}
	else
	{
				float nrm_old = sqrtf(belly_e_old*belly_e_old+belly_n_old*belly_n_old);

				float vexi1 = belly_n_old/nrm_old;
				float vexi2 = belly_e_old/nrm_old;

				float Wv1n = vexi2*fv3;
				float Wv2n = -vexi1*fv3;
				float Wv3n = fv2*vexi1 - fv1*vexi2;

				float _norm_Wn = sqrtf(Wv1n*Wv1n+Wv2n*Wv2n+Wv3n*Wv3n);
				float wv1 = Wv1n/_norm_Wn;
				float wv2 = Wv2n/_norm_Wn;
				float wv3 = 0.0f;

				float proj1 = wv3*fv2-wv2*fv3;
				float proj2 = wv1*fv3-wv3*fv1;
				float proj3 = wv2*fv1-wv1*fv2;

				float bldr_array_cri[9] = {fv1,wv1,proj1,fv2,wv2,proj2,fv3,wv3,proj3};
				matrix::SquareMatrix<float, 3> Bldr_Matrix_cri(bldr_array_cri);
				matrix::Dcmf C_ref_hover (Bldr_Matrix_cri); // DCM cast transposes?
				C_ri_pos = C_ref_hover;
	}
}

void FixedwingAttitudeControl::JUAN_singularity_management(float xy_speed, float angle_vect)
{
	// controller switch thresholds
	float angle_thrs_inf = 10*0.01745f;// 15 degrees
	float angle_thrs_sup = 20*0.01745f;// 25 degrees
	float vel_thrs_inf = 2.0f; // 2 m/s
	float vel_thrs_sup = 3.0f; // 5 m/s
	// Singularity management
	if (_control_operation_mode > 0)
	{ // in singularity mode, check if tilt and speed are enough
		if (xy_speed > vel_thrs_sup)
		{
			if (angle_vect > angle_thrs_sup)
			{
				_control_operation_mode = 0;
			}
			else{
				_control_operation_mode = 1;
			}
		}
		else{
			_control_operation_mode = 1;
		}
	}
	else //in normal mode
	{
		if (xy_speed > vel_thrs_inf)
		{
			if (angle_vect > angle_thrs_inf)
			{
				_control_operation_mode = 0;
			}
			else{
				_control_operation_mode = 1;
			}
		}
		else{
				_control_operation_mode = 1;
		}
	}
}

void FixedwingAttitudeControl::JUAN_reference_generator()
{

	if (_position_maneuver == 0)
	{
		if(_time_elapsed <= 2.0f)
		{
			JUAN_straight_line_time(10.0f, _initial_heading, 0.0f);
		}
		else
		{
			JUAN_straight_line_time(-10.0f, _initial_heading, 0.0f);
		}
	}

	else if (_position_maneuver == 1)
	{
		float _VelC = 5.0f;
		float _GaC = -15.0f*3.14f/180.0f;
		JUAN_straight_line_time(_VelC, _initial_heading, _GaC);
	// if (time_elapsed <= time_stage1)
	// {
			// float t_man = _time_elapsed;
			// float Vel_track1 = 10.0f;
	    // _vel_x_ref = Vel_track1*cosf(_initial_heading);
	    // _vel_y_ref = Vel_track1*sinf(_initial_heading);
	    // _vel_z_ref = 0.0f;
			//
	    // _pos_x_ref = _pos_x_initial+Vel_track1*cosf(_initial_heading)*t_man;
	    // _pos_y_ref = _pos_y_initial+Vel_track1*sinf(_initial_heading)*t_man;
	    // _pos_z_ref = _pos_z_initial;
	// }
	}
	else if (_position_maneuver == 2)
	{
		float t_man = _time_elapsed;
		float V_i = 8.0f;
		float t_init = 2.0;
		float t_stop = 2.0f;
		float a_slow = -V_i/t_stop;
		if (t_man < t_init)
		{
			_vel_x_ref = V_i*cosf(_initial_heading);
			_vel_y_ref = V_i*sinf(_initial_heading);
			_vel_z_ref = 0.0f;

			_pos_x_ref = _pos_x_initial+V_i*cosf(_initial_heading)*t_man;
	    _pos_y_ref = _pos_y_initial+V_i*sinf(_initial_heading)*t_man;
	    _pos_z_ref = _pos_z_initial;



		}
		else if (t_man < t_init+t_stop)
		{

			float vel_mag = a_slow*(t_man-t_init)+V_i;
			_vel_x_ref = vel_mag*cosf(_initial_heading);
			_vel_y_ref = vel_mag*sinf(_initial_heading);
			_vel_z_ref = 0.0f;

			float pos_mag = a_slow/2*(t_man-t_init)*(t_man-t_init)+V_i*(t_man-t_init)+V_i*t_init;
			_pos_x_ref = pos_mag*cosf(_initial_heading)+_pos_x_initial;
			_pos_y_ref = pos_mag*sinf(_initial_heading)+_pos_y_initial;
			_pos_z_ref = _pos_z_initial;

			float pos_mag_f = a_slow/2*t_stop*t_stop+V_i*t_stop+V_i*t_init;

   		_pos_x_prev = pos_mag_f*cosf(_initial_heading)+_pos_x_initial;
			_pos_y_prev = pos_mag_f*sinf(_initial_heading)+_pos_y_initial;

		}
		else {
			// // _vel_x_ref = 0.0f;
			// // _vel_y_ref = 0.0f;
			// // _vel_z_ref = 0.0f;
			//
			// float pos_mag = a_slow/2*t_stop*t_stop+V_i*t_stop+V_i*t_init;
			// // _pos_x_ref = pos_mag*cosf(_initial_heading)+_pos_x_initial;
			// // _pos_y_ref = pos_mag*sinf(_initial_heading)+_pos_y_initial;
			// // _pos_z_ref = _pos_z_initial;
			//
			// float _pos_x_ref_man0 = pos_mag*cosf(_initial_heading)+_pos_x_initial;
			// float _pos_y_ref_man0 = pos_mag*sinf(_initial_heading)+_pos_y_initial;
			// float _pos_z_ref_man0 = _pos_z_initial;


			float _comm_a = _read_roll_stick*5.0f;
			int _cast_comm_a = static_cast<int>(_comm_a);
			// _a_PF = 0.0f;
			_a_PF = -1.0f*_cast_comm_a/5.0f;

			float _comm_b = _read_pitch_stick*5.0f;
			int _cast_comm_b = static_cast<int>(_comm_b);
			// _b_PF = 0.0f;
			_b_PF = 1.0f*_cast_comm_b/5.0f;

			float _V_fb = _b_PF*4.0f;
			float _V_lat = -_a_PF*4.0f;

			_vel_x_ref = _V_fb*cosf(_initial_heading)-_V_lat*sinf(_initial_heading);
			_vel_y_ref = _V_fb*sinf(_initial_heading)+_V_lat*cosf(_initial_heading);
			_vel_z_ref = 0.0f;

			_pos_x_ref = _vel_x_ref*_delta_time_attitude + _pos_x_prev;
			_pos_y_ref = _vel_y_ref*_delta_time_attitude + _pos_y_prev;
			_pos_z_ref = _pos_z_initial;

			_pos_x_prev = _pos_x_ref;
			_pos_y_prev = _pos_y_ref;

		}


	}


}

void FixedwingAttitudeControl::JUAN_straight_line_time(float _Vel, float _Xi, float _Ga)
{
	float Vel_track1 = _Vel;
	_vel_x_ref = Vel_track1*cosf(_Xi)*cosf(_Ga);
	_vel_y_ref = Vel_track1*sinf(_Xi)*cosf(_Ga);
	_vel_z_ref = -Vel_track1*sinf(_Ga);

	_pos_x_ref = _delta_time_attitude*Vel_track1*cosf(_Xi)*cos(_Ga)+_pos_x_initial;
	_pos_y_ref = _delta_time_attitude*Vel_track1*sinf(_Xi)*cos(_Ga)+_pos_y_initial;
	_pos_z_ref = -_delta_time_attitude*Vel_track1*sinf(_Ga)+_pos_z_initial;

	_pos_x_initial = _pos_x_ref;
	_pos_y_initial = _pos_y_ref;
	_pos_z_initial = _pos_z_ref;
}

void FixedwingAttitudeControl::JUAN_helix_time(float _Vel, float _dir, float r, float c, float Xi)
{

	float _sigma_t = _Vel*_time_elapsed;

	float x = r*sinf(_sigma_t/sqrtf(r*r+c*c));
	float y = -math::sign(_dir)*(r-r*cosf(_sigma_t/sqrtf(r*r+c*c)));
	float z = -c*_sigma_t/sqrtf(r*r+c*c);

	float dx = _Vel*r/sqrtf(r*r+c*c)*cosf(_sigma_t/sqrtf(r*r+c*c));
	float dy = _Vel*-math::sign(_dir)*r/sqrtf(r*r+c*c)*sinf(_sigma_t/sqrtf(r*r+c*c));
	float dz = _Vel*-c/sqrtf(r*r+c*c);

	_pos_x_ref = (cosf(Xi)*x-sinf(Xi)*y)+_pos_x_initial;
	_pos_y_ref = (sinf(Xi)*x+cosf(Xi)*y)+_pos_y_initial;
	_pos_z_ref = z+_pos_z_initial;

	_vel_x_ref = (cosf(Xi)*dx-sinf(Xi)*dy);
	_vel_y_ref = (sinf(Xi)*dx+cosf(Xi)*dy);
	_vel_z_ref = z;

}


float FixedwingAttitudeControl::saturate(float value, float min, float max)
{
  float output = value;
  if (value < min){output = min;}
  if (value > max){output = max;}
  return output;
}

void FixedwingAttitudeControl::JUAN_enter_acro_manager()
{
	if (enter_acro < 1)
	{
		enter_acro = 1;
		flick_counter = flick_counter+1;
		// _juan_att_var.test_variable = flick_counter;
	}
	float _time_loop_start = hrt_absolute_time();
	float _time_attitude_now = hrt_absolute_time()/1e6;
	_delta_time_attitude = _time_attitude_now - _previous_time;
	_previous_time = _time_attitude_now;
	_time_elapsed = _time_elapsed + _delta_time_attitude;
}

void FixedwingAttitudeControl::JUAN_stabilized_reset()
{
	enter_acro = 0;
	if (flick_counter > 0)
	{
		_program_counter = 3;
	}
	else
	{
		_program_counter = 0;
	}
	_sigma_0 = 0.0f;
	_err_vel_x_int = 0.0f;
	_err_vel_y_int = 0.0f;
	_err_vel_z_int = 0.0f;

	_error_x_int = 0.0f;
	_error_y_int = 0.0f;
	_error_z_int = 0.0f;

	_Vel_PF_prev = 10.0f;
	_a_PF_prev = 0.0f;
	_b_PF_prev = 0.0f;
	_sigma = 0.0f;

}

void FixedwingAttitudeControl::JUAN_vehicle_states_reader()
{
	matrix::Dcmf Rr = matrix::Quatf(_att.q);
	C_bi = Rr.transpose();

	vehicle_angular_velocity_s angular_velocity{};
	_vehicle_rates_sub.copy(&angular_velocity);
	x_rate_body = angular_velocity.xyz[0];
	y_rate_body = angular_velocity.xyz[1];
	z_rate_body = angular_velocity.xyz[2];

	_read_roll_stick = _manual.y;
	_read_pitch_stick = _manual.x;
	_read_thrust_stick = _manual.z;
	_read_yaw_stick = _manual.r;

	// Assign measured position and velocity
	_local_pos_sub.update(&_local_pos);

	_pos_x_est = _local_pos.x;
	_pos_y_est = _local_pos.y;
	_pos_z_est = _local_pos.z;

	_vel_x_est = _local_pos.vx;
	_vel_y_est = _local_pos.vy;
	_vel_z_est = _local_pos.vz;
}

void FixedwingAttitudeControl::JUAN_reference_attitude_generator()
{
	_pos_x_ref = _pos_x_est;
	_pos_y_ref = _pos_y_est;
	_pos_z_ref = _pos_z_est;
	matrix::Eulerf euler_angles(C_bi.transpose());
	_heading_path = euler_angles.psi();
	if (_attitude_maneuver == 0)
	{
		/* ....................Manual attitude steady...........................*/

		if (fabsf(_read_roll_stick)>0.2)
		{
			_manual_roll = _read_roll_stick*0.785f;
		}
		else
		{
			_manual_roll = 0.0f;
		}
		if (fabsf(_read_pitch_stick)>0.2)
		{
			_manual_pitch = _read_pitch_stick*-0.785f;
		}
		else
		{
			_manual_roll = 0.0f;
		}
		float _extra_yaw_rate = 0.0f;
		if (fabsf(_read_yaw_stick)>0.5)
		{
			_extra_yaw_rate = _read_yaw_stick*1.0f;
		}


		// _manual_pitch = _read_pitch_stick*-0.785f;
		// float _extra_yaw_rate = _read_yaw_stick*1.5f;
		// // // Velocity
		//
		// matrix::Vector3f _inertial_velocity(_vel_x_est, _vel_y_est, _vel_z_est);
		// matrix::Vector3f _body_velocity = C_bi*_inertial_velocity;
		//
		// float _vel_ground = sqrtf(_body_velocity(0)*_body_velocity(0)+_body_velocity(1)+_body_velocity(1));
		// if (_vel_ground <5.0f){
		// 	_ground_velocity_corrected = 5.0f;
		// }
		// else {
		// 	_ground_velocity_corrected = _vel_ground;
		// }
		_ground_velocity_corrected = 10.0f;
		// JUAN note: tangent can cause issues if roll is allowed to go to 90 degrees!
		_heading_rate_coordinated = (1.0f*9.81f/_ground_velocity_corrected * tanf(_manual_roll))+_extra_yaw_rate;;
		// _heading_rate_coordinated = _heading_rate_coordinated+_extra_yaw_rate;
		//
		_manual_yaw = _delta_time_attitude*_heading_rate_coordinated+_previous_yaw;
		if (_manual_yaw < -PI_f)
		{
				_manual_yaw = _manual_yaw + 2*PI_f;
		}
		else if (_manual_yaw > PI_f)
		{
				_manual_yaw = _manual_yaw - 2*PI_f;
		}



		// bool _manual_yaw_check = PX4_ISFINITE(_manual_yaw);
		//
		// if (_manual_yaw_check)
		// {
			_previous_yaw = _manual_yaw;
		// }
		// else{
		// 	_manual_yaw = _previous_yaw;
		// }
		// _manual_yaw = _initial_heading;
		_roll_rate_reference = 0.0f;
		_pitch_rate_reference = 0.0f;
		_yaw_rate_reference = _heading_rate_coordinated;
		_juan_att_var.pilot_yaw_com = _manual_yaw;
		_juan_att_var.pilot_pitch_com = _manual_pitch;
		_juan_att_var.rate_ref_pitch = _pitch_rate_reference;
 	 _juan_att_var.rate_ref_roll = _roll_rate_reference;
 	 _juan_att_var.rate_ref_yaw = _yaw_rate_reference;


 }
 else if (_attitude_maneuver == 1) {
	 /* ....................Manual attitude hover ...........................*/
	 _manual_fourth = _read_roll_stick*0.6f;
	 _manual_pitch = 3.1415f/2.0f+_read_pitch_stick*-0.6f;
	 _manual_roll = 0.0f;




	 // JUAN note: tangent can cause issues if roll is allowed to go to 90 degrees!
	 float _yaw_rate_hover = 0.5f*_read_yaw_stick;

	 _manual_yaw = _delta_time_attitude*_yaw_rate_hover+_previous_yaw;
	 _previous_yaw = _manual_yaw;


	 // _manual_roll = 0.0f*3.1415f/180.0f;
	 // _manual_pitch = 90.0f*3.1415f/180.0f;
	 // _manual_yaw = _initial_heading;


	 _roll_rate_reference = 0.0f;
	 _pitch_rate_reference = 0.0f;
	 _yaw_rate_reference = _yaw_rate_hover;
	 // _yaw_rate_reference = 0.0f;
	 _heading_rate_coordinated = -1.0f;
	 _ground_velocity_corrected = -1.0f;
 }
 else if (_attitude_maneuver == 2) {
	 /* ........................ Loop profile ...........................*/
	 float _time_loop_man = _attitude_man_duration;

	 if (_time_elapsed < 2.0f){
	 	_pitch_test_profile = 0.0f;
	 	_pitch_rate_reference = 0.0f;
	 }
	 else if (_time_elapsed <2.0f+_time_loop_man){
	 	_pitch_rate_reference = 2.0f*3.1416f/_time_loop_man;
	 	_pitch_test_profile = _pitch_test_profile + _pitch_rate_reference*_delta_time_attitude;
	 }
	 else{
	 	_pitch_rate_reference = 0.0f;
	 	_pitch_test_profile = 0.0f;
	 }
	 _yaw_rate_reference = 0.0f;
	 _roll_rate_reference = 0.0f;

	 _juan_att_var.rate_ref_pitch = _pitch_rate_reference;
	 _juan_att_var.rate_ref_roll = _roll_rate_reference;
	 _juan_att_var.rate_ref_yaw = _yaw_rate_reference;

	  _manual_yaw = _locked_heading;
	 // _manual_yaw = _yaw_test_profile;
	 _manual_roll = _roll_test_profile;
	 _manual_pitch = _pitch_test_profile;
	 _juan_att_var.pilot_yaw_com = _manual_yaw;
		// _attitude_maneuver = 2+(flick_counter-1);
		// _position_control_flag = 0;
		// _thrust_hard_value = _manual.z;
		//
		// _thrust_hard_value = 1.0f;
	 _juan_att_var.pilot_pitch_com = _manual_pitch;
	 _juan_att_var.pilot_roll_com = _manual_roll;
	 _heading_rate_coordinated = -1.0f;
	 _ground_velocity_corrected = -1.0f;
 }
 else if (_attitude_maneuver == 3) {
	 /*..................  ATA profile   ................................*/

	 float _time_ata_man = _attitude_man_duration;
	 	if (_time_elapsed < 1.0f){
	 		_pitch_rate_reference = 0.0f;
	 	  _roll_rate_reference = 0.0f;
	 		_pitch_test_profile = 0.0f;
	 		_roll_test_profile = 0.0f;
	 	}
	 	else if (_time_elapsed <1.0f+_time_ata_man/2.0f){
	 		_pitch_rate_reference = 3.1416f/_time_ata_man;
	 		_pitch_test_profile = _pitch_test_profile + _pitch_rate_reference*_delta_time_attitude;
	 	 _roll_rate_reference = 0.0f;_juan_att_var.pilot_yaw_com = _manual_yaw;
		_juan_att_var.pilot_pitch_com = _manual_pitch;
		_juan_att_var.rate_ref_pitch = _pitch_rate_reference;
 	 _juan_att_var.rate_ref_roll = _roll_rate_reference;
 	 _juan_att_var.rate_ref_yaw = _yaw_rate_reference;

	 		_roll_test_profile = 0.0f;
	 	}
	 	else if(_time_elapsed <1.0f+_time_ata_man){
	 		_pitch_rate_reference = 3.1416f/_time_ata_man;
	 		_roll_rate_reference = 2.0f*3.1416f/_time_ata_man;
	 		_pitch_test_profile = _pitch_test_profile + _pitch_rate_reference*_delta_time_attitude;
	 		_roll_test_profile = _roll_test_profile + _roll_rate_reference*_delta_time_attitude;
	 	}
	 	else {
	 	 _pitch_rate_reference = 0.0f;
	 	 _roll_rate_reference = 0.0f;
	 		_pitch_test_profile = 3.1416f;
	 		_roll_test_profile = 3.1415f;
	 	}

	 	_yaw_rate_reference = 0.0f;
	 	// _manual_yaw = _yaw_test_profile;
		_manual_yaw = _locked_heading;
	 	_manual_roll = _roll_test_profile;
	 	_manual_pitch = _pitch_test_profile;
	 	_heading_rate_coordinated = -1.0f;
	 	_ground_velocity_corrected = -1.0f;
		_juan_att_var.pilot_roll_com = _manual_roll;
		_juan_att_var.pilot_yaw_com = _manual_yaw;
		_juan_att_var.pilot_pitch_com = _manual_pitch;
		_juan_att_var.rate_ref_pitch = _pitch_rate_reference;
 	 _juan_att_var.rate_ref_roll = _roll_rate_reference;
 	 _juan_att_var.rate_ref_yaw = _yaw_rate_reference;

 }
 else if (_attitude_maneuver == 4) {
	 /*.......................Sudden pitch SO3 error functions test......*/
	 if (_time_elapsed < 2.0f){
	 	_pitch_test_profile = 0.1745f;
	 	_pitch_rate_reference = 0.0f;
	 }
	 else if (_time_elapsed < 4.0f){
	 	_pitch_rate_reference = 0.0f;
	 	_pitch_test_profile = 2.9671f;
	 }
	 else{
		//  _attitude_maneuver = 3;
		// // _attitude_maneuver = 2+(flick_counter-1);
		// _position_control_flag = 0;
		// // _thrust_hard_value = _manual.z;
		// //
		// _thrust_hard_value = 1.0f;
	 	_pitch_test_profile = 0.1745f;
	 	_pitch_rate_reference = 0.0f;
	 }
	 _yaw_rate_reference = 0.0f;
	 _roll_rate_reference = 0.0f;

	 _manual_yaw = _yaw_test_profile;
	 _manual_roll = _roll_test_profile;
	 _manual_pitch = _pitch_test_profile;
	 _heading_rate_coordinated = -1.0f;
	 _ground_velocity_corrected = -1.0f;
 }
 else if(_attitude_maneuver == 5)
 {
	 /*..................Takeoff attitude .......................*/
	 _manual_yaw = _yaw_test_profile;
	 _manual_roll = 0.0f;
	 _manual_pitch = 0.7853f;

	 _yaw_rate_reference = 0.0f;
	 _roll_rate_reference = 0.0f;
	 _pitch_rate_reference = 0.0f;
	 _heading_rate_coordinated = -1.0f;
	 _ground_velocity_corrected = -1.0f;

 }
 else if(_attitude_maneuver == 6)
 {
	 /*..................Neutral attitude .......................*/
	 _manual_yaw = _yaw_test_profile;
	 _manual_roll = 0.0f;
	 _manual_pitch = 0.0f;

	 _yaw_rate_reference = 0.0f;
	 _roll_rate_reference = 0.0f;
	 _pitch_rate_reference = 0.0f;
	 _heading_rate_coordinated = -1.0f;
	 _ground_velocity_corrected = -1.0f;

 }

 else if (_attitude_maneuver == 7) {
	 // /*.......................Four 90 rolls......*/
	 // if (_time_elapsed < 1.5f){
	 // 	_roll_test_profile = 0.0f;
	 // }
	 // else if (_time_elapsed < 3.0f){
	 // 	_roll_test_profile = 1.5708f;
	 // }
	 // else if (_time_elapsed < 4.5f){
	 // 	_roll_test_profile = 3.1416f;
	 // }
	 // else if (_time_elapsed < 6.0f){
	 // 	_roll_test_profile = -1.5708f;
	 // }
	 // else
	 // {
	 // 	_roll_test_profile = 0.0f;
	 // }
	 /* Sudden pitch and roll */

	 if (_time_elapsed < 1.5f)
	 {
		 _pitch_test_profile = 10.0f*3.1516f/180.0f;
		 _roll_test_profile = 0.0f;

	 }
	 else if (_time_elapsed < 2.5f)
	 {
		 _pitch_test_profile = 10.0f*3.1516f/180.0f;
		 _roll_test_profile = 178.0f*3.1516f/180.0f;
	 }
	 else
	 {
		 _pitch_test_profile = 10.0f*3.1516f/180.0f;
		 _roll_test_profile = 0.0f;
	 }


	 _roll_rate_reference = 0.0f;
	 _pitch_rate_reference = 0.0f;
	 _yaw_rate_reference = 0.0f;

	 // _manual_yaw = _yaw_test_profile;
	 _manual_yaw = _locked_heading;
	 _manual_roll = _roll_test_profile;
	 _manual_pitch = _pitch_test_profile;
	 _heading_rate_coordinated = -1.0f;
	 _ground_velocity_corrected = -1.0f;
	 _juan_att_var.pilot_roll_com = _manual_roll;
	 _juan_att_var.pilot_yaw_com = _manual_yaw;
	 _juan_att_var.pilot_pitch_com = _manual_pitch;
	 _juan_att_var.rate_ref_pitch = _pitch_rate_reference;
		_juan_att_var.rate_ref_roll = _roll_rate_reference;
		_juan_att_var.rate_ref_yaw = _yaw_rate_reference;
 }
 else if (_attitude_maneuver == 8) {
	 /* ........................ Barrel roll profile ...........................*/

	 if (_time_elapsed < 2.0f){
	 	_roll_test_profile = 0.0f;
	 	_roll_rate_reference = 0.0f;
	 }
	 else {
	 	_roll_rate_reference = 4.5f;
	 	_roll_test_profile = _roll_test_profile + _roll_rate_reference*_delta_time_attitude;
	 }
	 _yaw_rate_reference = 0.0f;
	 _pitch_rate_reference = 0.0f;

	 _juan_att_var.rate_ref_pitch = _pitch_rate_reference;
	 _juan_att_var.rate_ref_roll = _roll_rate_reference;
	 _juan_att_var.rate_ref_yaw = _yaw_rate_reference;
	  _manual_yaw = _locked_heading;
	 // _manual_yaw = _yaw_test_profile;
	 _manual_roll = _roll_test_profile;
	 // _manual_pitch = _pitch_test_profile;
	 _manual_pitch = 10.0f*3.1416f/180.0f;
	 _juan_att_var.pilot_yaw_com = _manual_yaw;
		// _attitude_maneuver = 2+(flick_counter-1);
		// _position_control_flag = 0;
		// _thrust_hard_value = _manual.z;
		//
		// _thrust_hard_value = 1.0f;
	 _juan_att_var.pilot_pitch_com = _manual_pitch;
	 _juan_att_var.pilot_roll_com = _manual_roll;
	 _heading_rate_coordinated = -1.0f;
	 _ground_velocity_corrected = -1.0f;
 }


 else if (_attitude_maneuver == 9) {
	 /*.......................Arbitrary axis maneuver no alpha......*/
	 float Tt_man = 2.0f;
	 float Tt_wait = 2.0f;

	 if (_time_elapsed < Tt_wait){
	 	mc_alpha = 0.0f;
		d_mc_alpha = 0.0f;
	}
	 else if(_time_elapsed <Tt_wait+Tt_man){
		 d_mc_alpha = 2*3.1416f/Tt_man;
		 mc_alpha = d_mc_alpha*(_time_elapsed-Tt_wait);
	 }
	 else{
		 mc_alpha = 0.0f;
 		 d_mc_alpha = 0.0f;
	 }
 }

 else if (_attitude_maneuver == 10) {
	/*.......................Arbitrary axis maneuver with alpha 5th order......*/
	float Tt_man = 2.0f;
	float Tt_wait = 2.0f;

	float a3 = 1.0f/(Tt_man*Tt_man*Tt_man)*20.0f*3.1516f;
	float a4 = -1.0f/(Tt_man*Tt_man*Tt_man*Tt_man)*30.0f*3.1516f;
  float a5 = 1.0f/(Tt_man*Tt_man*Tt_man*Tt_man*Tt_man)*12.0f*3.1516f;

	if (_time_elapsed < Tt_wait){
	 mc_alpha = 0.0f;
	 d_mc_alpha = 0.0f;
	 dd_mc_alpha = 0.0f;
 }
	else if(_time_elapsed <Tt_wait+Tt_man){
		float Ttm = _time_elapsed-Tt_wait;
		mc_alpha = a3*Ttm*Ttm*Ttm+a4*Ttm*Ttm*Ttm*Ttm+a5*Ttm*Ttm*Ttm*Ttm*Ttm;
    d_mc_alpha = 3.0f*a3*Ttm*Ttm+4.0f*a4*Ttm*Ttm*Ttm+5.0f*a5*Ttm*Ttm*Ttm*Ttm;
		dd_mc_alpha = 6.0f*a3*Ttm+12.0f*a4*Ttm*Ttm+20.0f*a5*Ttm*Ttm*Ttm;
	}
	else{
		mc_alpha = 0.0f;
		d_mc_alpha = 0.0f;
		dd_mc_alpha = 0.0f;
	}
 }













 // if (_attitude_maneuver == 1 ) {
	//  float bldr_array_Cfourth[9] = {cosf(_manual_fourth), sinf(_manual_fourth), 0.0f, -sinf(_manual_fourth), cosf(_manual_fourth), 0.0f, 0.0f, 0.0f, 1.0f};
 //   matrix::SquareMatrix<float, 3> Bldr_Matrix_Cfourth(bldr_array_Cfourth);
 //   matrix::Dcmf C_fourth(Bldr_Matrix_Cfourth);
	//  matrix::Dcmf C_manual = C_fourth*C_roll*C_pitch*C_yaw;
	//   // matrix::Dcmf C_manual = C_roll*C_pitch*C_yaw;
	//  C_ri = C_manual;
 // }
 // else{
 if (_attitude_maneuver<9){
	 /*..DCMs for principal rotations.*/
	 float bldr_array_Cyaw[9] = {cosf(_manual_yaw), sinf(_manual_yaw), 0.0f, -sinf(_manual_yaw), cosf(_manual_yaw), 0.0f, 0.0f, 0.0f, 1.0f};
	 matrix::SquareMatrix<float, 3> Bldr_Matrix_Cyaw(bldr_array_Cyaw);
	 matrix::Dcmf C_yaw(Bldr_Matrix_Cyaw);

	 float bldr_array_Cpitch[9] = {cosf(_manual_pitch), 0.0f, -sinf(_manual_pitch), 0.0f, 1.0f, 0.0f, sinf(_manual_pitch), 0.0f, cosf(_manual_pitch)};
	 matrix::SquareMatrix<float, 3> Bldr_Matrix_Cpitch(bldr_array_Cpitch);
	 matrix::Dcmf C_pitch(Bldr_Matrix_Cpitch);

	 float bldr_array_Croll[9] = {1.0f, 0.0f, 0.0f, 0.0f, cosf(_manual_roll), sinf(_manual_roll), 0.0f, -sinf(_manual_roll), cosf(_manual_roll)};
	 matrix::SquareMatrix<float, 3> Bldr_Matrix_Croll(bldr_array_Croll);
	 matrix::Dcmf C_roll(Bldr_Matrix_Croll);
	 matrix::Dcmf C_manual = C_roll*C_pitch*C_yaw;
	 C_ri = C_manual;
 // }
		matrix::Dcmf C_riT = C_ri.transpose();
		matrix::Dcmf C_br_alt = C_bi*C_riT;
		matrix::Vector3f _reference_euler_rate(_roll_rate_reference,_pitch_rate_reference,_yaw_rate_reference);
		float bldr_array_e2w[9] = {1.0f, 0.0f, -sinf(_pitch_test_profile), 0.0f, cosf(_roll_test_profile), sinf(_roll_test_profile)*cosf(_pitch_test_profile), 0.0f, -sinf(_roll_test_profile), cosf(_roll_test_profile)*cosf(_pitch_test_profile)};
		matrix::SquareMatrix<float, 3> Bldr_Matrix_Ce2w(bldr_array_e2w);
		matrix::Dcmf C_e2w(Bldr_Matrix_Ce2w);
		matrix::Vector3f _omega_reference = C_e2w*_reference_euler_rate; // reference angular velocity in reference coord
		// _omega_reference_body =  C_br_alt*_omega_reference; // reference angular velocity in body coord
		_omega_reference_body =  _omega_reference;
		matrix::Vector3f _alpha_ref_temp(0.0f, 0.0f, 0.0f);
		_alpha_reference_body = _alpha_ref_temp;
	}
	else{
		float k1 = 1.0f/sqrtf(2.0f);
		float k2 = 1.0f/sqrtf(2.0f);
		float k3 = 0.0f/sqrtf(2.0f);

		float _cos_a = cosf(mc_alpha);
		float _sin_a = sinf(mc_alpha);

		float bldr_array_Calp[9] = {_cos_a+(1-_cos_a)*k1*k1, (1-_cos_a)*k1*k2+_sin_a*k3, (1-_cos_a)*k1*k3-_sin_a*k2, (1-_cos_a)*k1*k2-_sin_a*k3, (1-_cos_a)*k2*k2+_cos_a, (1-_cos_a)*k2*k3+_sin_a*k1, (1-_cos_a)*k1*k3+_sin_a*k2, (1-_cos_a)*k2*k3-_sin_a*k1, (1-_cos_a)*k3*k3+_cos_a};
		matrix::SquareMatrix<float, 3> Bldr_Matrix_Calp(bldr_array_Calp);
		matrix::Dcmf C_alpha(Bldr_Matrix_Calp);

		C_ri = C_alpha;
    matrix::Vector3f _omega_temp(d_mc_alpha*k1, d_mc_alpha*k2, d_mc_alpha*k3);
		matrix::Vector3f _alpha_temp(dd_mc_alpha*k1, dd_mc_alpha*k2, dd_mc_alpha*k3);
		_omega_reference_body = _omega_temp;
		_alpha_reference_body = _alpha_temp;
	}



}

void FixedwingAttitudeControl::JUAN_control_allocation()
{
	/*.........Thrust Mapping ..............................................*/
	if (_position_control_flag == 1)
	{
		float Jar = 0.5f;

		/*..........Proper advance ratio calculation........................*/

		// if (_previous_rpm < 1000)
		// {
		// 	_previous_rpm = 1000.0f;
		// }
		// float _norm_vel =  sqrtf(_local_pos.vx*_local_pos.vx+_local_pos.vy*_local_pos.vy+_local_pos.vz*_local_pos.vz);
		// _advance_ratio = _norm_vel/(0.254f*(_previous_rpm/60.0f));
		//
		// float Jar = saturate(_advance_ratio,0.0f,0.5f);
		/*..................................................................*/


		_global_jar = Jar;

		float kt = (-1.43909969 * Jar * Jar - 2.21240323 * Jar + 2.24512051) * powf(10.0f,-7.0f);
		float omega_t = sqrtf(ThrustN/kt);
		_previous_rpm = omega_t;
		float thrust_PWM = saturate(1.6572 * powf(10.0f,-5.0f) * powf(omega_t,2.0) + .0166 * omega_t + 1121.8,1000,2000);
		_throttle_out = (thrust_PWM-1000)/1000;
	}
	else {
			// _throttle_out = saturate(_thrust_hard_value, -1.0f, 1.0f);
			_throttle_out = _thrust_hard_value;
	}
	float outputs_ail = 0.0000016235f*powf(_AilDef,3.0f) - 0.0000009861f*powf(_AilDef,2.0f) + 0.0145866432f*_AilDef;
	float outputs_ele = 0.0000008317f*powf(_ElevDef,3.0f) + 0.0000409759f*powf(_ElevDef,2.0f) + 0.01396963f*_ElevDef;
	float outputs_rud = 0.0000007988f*powf(_RudDef,3.0f) + 0.0000092020f*powf(_RudDef,2.0f) + 0.0187045418f*_RudDef;

	/*........................... PX4 actuator mappings.................*/
	_actuators.control[actuator_controls_s::INDEX_ROLL] = -outputs_ail;
	_actuators.control[actuator_controls_s::INDEX_PITCH] = -outputs_ele;
	_actuators.control[actuator_controls_s::INDEX_YAW] = -outputs_rud;
	_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _throttle_out;
	/*..................................................................*/

}

void FixedwingAttitudeControl::JUAN_mission_planner()
{

	if (_program_counter == 0)
	{
		_attitude_maneuver = 6;
		_position_control_flag = 0;
		_thrust_hard_value = 0.0f;
		if (_time_elapsed>3.0f)
		{
			_program_counter = 1;
		}
	}
	else if (_program_counter == 1) {
		_attitude_maneuver = 5;
		_position_control_flag = 0;
		_thrust_hard_value = 0.0f;
		if (_manual.z > 0.8f){
			_program_counter = 2;
		}
	}
	else if (_program_counter == 2) {
		_attitude_maneuver = 5;
		_position_control_flag = 0;
		_thrust_hard_value = 0.7f;
		if (fabsf(_pos_z_est)>30.0f){
			_program_counter = 3;
			_control_mode_change_direction = 1;
			JUAN_pose_initialize();
		}
	}
	else
	{
		// _position_control_flag = 0;
		// // JUAN_loop_experiments();
		// // JUAN_slanted_loop_experiments();
		// // JUAN_ATA_experiments();
		// // JUAN_harrier_experiments();
		// JUAN_sudden_experiments();



		// _position_control_flag = 0;
		// _attitude_maneuver = 0;
		// // _thrust_hard_value = _manual.z;
		// // _attitude_maneuver = 2+(flick_counter-1);
		//
		// // _thrust_hard_value = _manual.z;
		// //
		// _thrust_hard_value = 1.0f;



		/* Manual hover = 2*/
		// _position_control_flag = 1;
		// _velocity_control_flag = 0;
		// _path_following_flag = 0;
		// _position_maneuver = 2 ;
		// JUAN_reference_generator();


		// _position_control_flag = 1;
		// _path_following_flag = 1;
		// _Vel_PF = 10.0f;
		// _a_PF = 0.0f;
		// _b_PF = 0.5f;

		// _position_control_flag = 1;
		// _path_following_flag = 0;
		// _velocity_control_flag = 0;
		// // JUAN_straight_line_time(10.0f, _initial_heading, 0.0f);
		// JUAN_helix_time(10.0f, -1.0f, 10.0f, -2.0f, _initial_heading);
		//
		// _position_maneuver = 2;
		// JUAN_reference_generator();

		// _position_control_flag = 1;
		// _path_following_flag = 1;
		// _velocity_control_flag = 1;
		// JUAN_provisional_path_following();

		// //
		_path_following_flag = 1;
		JUAN_manual_PF_manager();

	}
}

void FixedwingAttitudeControl::JUAN_logger()
{
	/*...................Custom logging ................................*/
	/*...............Always logged......................................*/
	_juan_att_var.timestamp = hrt_absolute_time();

	_juan_att_var.test_variable = _probe_var;

	_juan_att_var.tau_ref[0] = _tau_1;
	_juan_att_var.tau_ref[1] = _tau_2;
	_juan_att_var.tau_ref[2] = _tau_3;

	_juan_att_var.innov_term = _attitude_error_innovation;
	_juan_att_var.att_man_id = _attitude_maneuver;

	/*These error variables are saved on the attitude control routine*/
	// _juan_att_var.error_so3[0] = err_att_1;
	// _juan_att_var.error_so3[1] = err_att_2;
	// _juan_att_var.error_so3[2] = err_att_3;


	_juan_att_var.ctr_defl[0] = _AilDef;
	_juan_att_var.ctr_defl[1] = _ElevDef;
	_juan_att_var.ctr_defl[2] = _RudDef;
	_juan_att_var.normalized_throttle = _throttle_out;
	//
	// // _juan_att_var.pwm_out_ctr[0] = outputs_ail;
	// // _juan_att_var.pwm_out_ctr[1] = outputs_ele;
	// // _juan_att_var.pwm_out_ctr[2] = outputs_rud;

	// _juan_att_var.ground_speed_attitude = _ground_velocity_corrected;
	// _juan_att_var.coordinate_yaw_rate = _heading_rate_coordinated;
	// _juan_att_var.attitude_error_function = att_err_function;
	//
	//
	_juan_att_var.omega_ref[0] = _omega_reference_body(0);
	_juan_att_var.omega_ref[1] = _omega_reference_body(1);
	_juan_att_var.omega_ref[2] = _omega_reference_body(2);
	//
	_juan_att_var.omega_bod[0] = x_rate_body;
	_juan_att_var.omega_bod[1] = y_rate_body;
	_juan_att_var.omega_bod[2] = z_rate_body;
	//
	_juan_att_var.cbi_rows[0] = C_bi(0,0);
	_juan_att_var.cbi_rows[1] = C_bi(0,1);
	_juan_att_var.cbi_rows[2] = C_bi(0,2);
	_juan_att_var.cbi_rows[3] = C_bi(1,0);
	_juan_att_var.cbi_rows[4] = C_bi(1,1);
	_juan_att_var.cbi_rows[5] = C_bi(1,2);
	_juan_att_var.cbi_rows[6] = C_bi(2,0);
	_juan_att_var.cbi_rows[7] = C_bi(2,1);
	_juan_att_var.cbi_rows[8] = C_bi(2,2);
	//
	_juan_att_var.cri_rows[0] = C_ri(0,0);
	_juan_att_var.cri_rows[1] = C_ri(0,1);
	_juan_att_var.cri_rows[2] = C_ri(0,2);
	_juan_att_var.cri_rows[3] = C_ri(1,0);
	_juan_att_var.cri_rows[4] = C_ri(1,1);
	_juan_att_var.cri_rows[5] = C_ri(1,2);
	_juan_att_var.cri_rows[6] = C_ri(2,0);
	_juan_att_var.cri_rows[7] = C_ri(2,1);
	_juan_att_var.cri_rows[8] = C_ri(2,2);


	_juan_att_var.estimated_position_x = _pos_x_est;
	_juan_att_var.estimated_position_y = _pos_y_est;
	_juan_att_var.estimated_position_z = _pos_z_est;

	_juan_att_var.read_pitch = _read_pitch_stick;
	_juan_att_var.read_roll = _read_roll_stick;
	_juan_att_var.read_yaw = _read_yaw_stick;
	_juan_att_var.read_thrust = _read_thrust_stick;

	_juan_att_var.slipstr_att = _slips_estimate;



	if (_position_control_flag == 1) {

		_juan_att_var.reference_position_x = _pos_x_ref;
		_juan_att_var.reference_position_y = _pos_y_ref;
		_juan_att_var.reference_position_z = _pos_z_ref;


		_juan_att_var.estimated_velocity_x = _vel_x_est;
		_juan_att_var.estimated_velocity_y = _vel_y_est;
		_juan_att_var.estimated_velocity_z = _vel_z_est;

		_juan_att_var.reference_velocity_x = _vel_x_ref;
		_juan_att_var.reference_velocity_y = _vel_y_ref;
		_juan_att_var.reference_velocity_z = _vel_z_ref;

		_juan_att_var.control_status = _control_operation_mode;
		_juan_att_var.virtual_force[0] = fv1;
		_juan_att_var.virtual_force[1] = fv2;
		_juan_att_var.virtual_force[2] = fv3;
		_juan_att_var.lateral_err = _lateral_error_roll;

		if(_path_following_flag == 1)
		{
			// _juan_att_var.path_law = _d_sigma;
			_juan_att_var.tangent_v1 = _Tv1;
			_juan_att_var.tangent_v2 = _Tv2;
			_juan_att_var.tangent_v3 = _Tv3;

			// _juan_att_var.sigma_path = _sigma;
			_juan_att_var.error_s = _error_onpath;
			_juan_att_var.error_c = _error_crosstrack;
			_juan_att_var.error_h = _error_altitude;

		}

	}


	// _juan_att_var.test_variable = _throttle_out;


	// // matrix::Eulerf euler_ref(C_ri.transpose());
	// // _juan_att_var.yaw_reference = euler_ref.psi();
	// // _juan_att_var.pitch_reference = euler_ref.theta();
	// // _juan_att_var.roll_reference = euler_ref.phi();
	//
	//
	// // float _time_loop_end = hrt_absolute_time();
	// // float _delta_run_time = _time_loop_end-_time_loop_start;
	// //
	// // _juan_att_var.test_variable = _delta_run_time;
	// //
	//
	//
	// // JUAN_reference_generator(1);
	// if (_JUAN_flight_mode > 0)
	// {
	// 	_juan_att_var.reference_position_x = _pos_x_ref;
	// 	_juan_att_var.reference_position_y = _pos_y_ref;
	// 	_juan_att_var.reference_position_z = _pos_z_ref;
	//
	// 	// _local_pos_sub.update(&_local_pos);
	// 	float _pos_x_est = _local_pos.x;
	// 	float _pos_y_est = _local_pos.y;
	// 	float _pos_z_est = _local_pos.z;
	//
	// 	//

	//
	// 	float _vel_x_log = _local_pos.vx;
	// 	float _vel_y_log = _local_pos.vy;
	// 	float _vel_z_log = _local_pos.vz;
	//
	// 	_juan_att_var.estimated_velocity_x = _vel_x_log;
	// 	_juan_att_var.estimated_velocity_y = _vel_y_log;
	// 	_juan_att_var.estimated_velocity_z = _vel_z_log;
	//
	//
	// 	_juan_att_var.control_status = _control_operation_mode;innovation
	// 	_juan_att_var.j_advance = _global_jar;
	// 	_juan_att_var.omega_rpm = _previous_rpm;
	//
	//
	// 	_juan_att_var.virtual_force[0] = fv1;
	// 	_juan_att_var.virtual_force[1] = fv2;
	// 	_juan_att_var.virtual_force[2] = fv3;
	//
	//
	// }
	// else {
	// 	_juan_att_var.reference_position_x = _pos_x_est;
	// 	_juan_att_var.reference_position_y = _pos_y_est;
	// 	_juan_att_var.reference_position_z = _pos_z_est;
	//
	// 	//
	// 	_juan_att_var.estimated_position_x = _pos_x_est;
	// 	_juan_att_var.estimated_position_y = _pos_y_est;
	// 	_juan_att_var.estimated_position_z = _pos_z_est;
	//
	// 	_juan_att_var.estimated_velocity_x = _local_pos.vx;
	// 	_juan_att_var.estimated_velocity_y = _local_pos.vy;
	// 	_juan_att_var.estimated_velocity_z = _local_pos.vz;
	//
	// }
	//
	//
	//
	//
	//
	// _juan_att_var.heading_0 = _initial_heading;
	//

	_juan_attitude_variables_pub.publish(_juan_att_var);
	/*..................................................................*/

}

void FixedwingAttitudeControl::JUAN_attitude_control(int _innovation_option)
{
	matrix::Dcmf C_ir = C_ri.transpose();
	matrix::Dcmf C_br = C_bi*C_ir;

	 /*.......SO(3) error calculation (and definition)...................*/
	 float att_err_function = _innovation_option;


	 if (att_err_function == 0)
	 {
		 att_err_modifier = 0.5f; // This is the standard
	 }
	 else{
		 float trace_C = C_br(0,0) + C_br(1,1) + C_br(2,2);
		 float err_att_aux = 1+trace_C;
		 if (err_att_aux < 0.01)
		 {
			 att_err_modifier = 1.0f;
		 }
		 else
		 {
			 if (att_err_function == 1){
			 		att_err_modifier = 1.0f/sqrtf(1+trace_C); //t. lee
		 	 }
			 else{
				 att_err_modifier = 2.0f/(1+trace_C); //forbes
			 }
		 }
	 }


	 float err_att_1 = att_err_modifier*-0.5f*(C_br(1,2)-C_br(2,1));
	 float err_att_2 = att_err_modifier*0.5f*(C_br(0,2)-C_br(2,0));
	 float err_att_3 = att_err_modifier*-0.5f*(C_br(0,1)-C_br(1,0));

	 _juan_att_var.error_so3[0] = err_att_1;
	 _juan_att_var.error_so3[1] = err_att_2;
	 _juan_att_var.error_so3[2] = err_att_3;
	 /*..................................................................*/

	 /* Constants used by control system ................................*/
	 float S_area = 0.14274f; //Wing Area (m^2), yak54 = 0.14865
	 float b_span = 0.864f; //Wing Span (m), yak54 = .82
	 float c_bar = 0.21f; //Mean Aerodynamic Chord (m), yak54 =.2107
	 float Cl_delta_a = -0.0006777f; //Aileron Control Derivative Coefficient (/deg)
	 float Cm_delta_e = -0.0117747f; //Elevator Control Derivative Coefficient (/deg)
	 float Cn_delta_r = -0.0035663f; //Rudder Control Derivative Coefficient (/deg)
	 float ro = 1.225f;

	 float AilDef_max = 52.0f; //52.0fMaximum Aileron Deflection (deg)
	 float ElevDef_max = 59.0f; //35.0fMaximum Elevator Deflection (deg)
	 float RudDef_max = 49.0f; //56.0fMaximum Rudder Deflection (deg)
	 float omega_t_max = 6710.0f; //Maximimum Thrust (RPM)
	 float omega_t_min = 1716.0f; //Minimum Thrust (RPM)
	 /*..................................................................*/

	 /*................Attitude controller gains Final Experiments..............*/
	 // float Kad1 = 0.8f*0.8f*0.8f*0.00706f;
	 // float Kad2 = 0.6f*0.5f*0.07576f;
	 // float Kad3 = 0.7f*0.07736f;
	 //
	 // float Kap1 = 0.8f*1.3f*0.1656f;
	 // float Kap2 = 0.6f*1.3f*1.0f*1.022f;
	 // float Kap3 = 1.0f*0.6776f;
	 //
	 // float Kai1 = 1.1f*0.8f*0.1656f;
	 // float Kai2 = 1.1f*0.8f*1.022f;
	 // float Kai3 = 1.0f*0.8f*0.6776f;
	 /*..................................................................*/


	 /*................Attitude controller gains Trial SITL..............*/
	 float Kad1 = 1.6f*0.8f*0.8f*0.00706f;
	 float Kad2 = 1.2f*0.5f*0.07576f;
	 float Kad3 = 1.4f*0.07736f;

	 float Kap1 = 1.6f*1.3f*0.1656f;
	 float Kap2 = 1.2f*1.3f*1.0f*1.022f;
	 float Kap3 = 1.4f*0.6776f;

	float Kai1 = 0.0f;
	float Kai2 = 0.0f;
	float Kai3 = 0.0f;



	 // /* Integral errors */
	 _e_int_1 = _e_int_1 + _delta_time_attitude*_e_int_1;
	 _e_int_2 = _e_int_2 + _delta_time_attitude*_e_int_2;
	 _e_int_3 = _e_int_3 + _delta_time_attitude*_e_int_3;
	 /*..................................................................*/


	 /*.... SO(3) attitude control law ..................................*/

	 // float tau_1 = -0.7f*Kad1*x_rate_body+0.8f*Kap1*err_att_1;
	 // float tau_2 = -0.8f*Kad2*y_rate_body+1.0f*Kap2*err_att_2;
	 // float tau_3 = -0.8f*Kad3*z_rate_body+0.8f*Kap3*err_att_3;

	 // float tau_1 = -0.7f*Kad1*(x_rate_body-_omega_reference_body(0))+0.8f*Kap1*err_att_1;
	 // float tau_2 = -0.8f*Kad2*(y_raVste_body-_omega_reference_body(1))+0.8f*Kap2*err_att_2;
	 // float tau_3 = -0.8f*Kad3*(z_rate_body-_omega_reference_body(2))+0.8f*Kap3*err_att_3;


	 // float tau_1 = -0.7f*Kad1*x_rate_body+0.8f*Kap1*err_att_1+Kai1*_e_int_1;
	 // float tau_2 = -0.8f*Kad2*y_rate_body+1.0f*Kap2*err_att_2+Kai2*_e_int_2;
	 // float tau_3 = -0.8f*Kad3*z_rate_body+0.8f*Kap3*err_att_3+Kai3*_e_int_3;


	 /*................. Final one!!!! ...................................*/
	 // float tau_1 = -0.7f*Kad1*(x_rate_body-_omega_reference_body(0))+0.8f*Kap1*err_att_1+Kai1*_e_int_1;
	 // float tau_2 = -0.8f*Kad2*(y_rate_body-_omega_reference_body(1))+1.0f*Kap2*err_att_cww	2+Kai2*_e_int_2;
	 // float tau_3 = -0.8f*Kad3*(z_rate_body-_omega_reference_body(2))+0.8f*Kap3*err_att_3+Kai3*_e_int_3;
	 if (_PD_plus_flag == 0){
	 _tau_1 = -0.7f*Kad1*(x_rate_body-_omega_reference_body(0))+1.2f*0.8f*Kap1*err_att_1+Kai1*_e_int_1;
	 _tau_2 = -0.6f*Kad2*(y_rate_body-_omega_reference_body(1))+1.2f*0.8f*Kap2*err_att_2+Kai2*_e_int_2;
	 _tau_3 = -0.8f*Kad3*(z_rate_body-_omega_reference_body(2))+1.2f*0.8f*Kap3*err_att_3+Kai3*_e_int_3;
 }


/*.........Alternative Tracking control law ..................................*/
else{

					/*.......Tracking law attitude controller gains.....................*/
					float la1 = 2.0f;
					float la2 = 2.0f;
					float la3 = 2.0f;

					float Kapp1 = Kap1 - la1*Kad1;
					float Kapp2 = Kap2 - la2*Kad2;
					float Kapp3 = Kap3 - la3*Kad3;

					/*.......Angular velocity error and composite error.................*/
					matrix::Vector3f _error_attitude(err_att_1,err_att_2,err_att_3);
					matrix::Vector3f _omega_estimate(x_rate_body,y_rate_body,z_rate_body);
					matrix::Vector3f _omega_tilde(0.0f,0.0f,0.0f);
					_omega_tilde = _omega_estimate - _omega_reference_body;
					matrix::Vector3f _sigma_error(_omega_tilde(0)-la1*err_att_1,_omega_tilde(1)-la2*err_att_2,_omega_tilde(2)-la3*err_att_3);
					matrix::Vector3f _omega_aux(_omega_reference_body(0)+la1*err_att_1,_omega_reference_body(1)+la2*err_att_2,_omega_reference_body(2)-la3*err_att_3);


					/*.....Control law..................................................*/
					float bldr_array_inertia[9] = {0.003922f, 0.0f, -0.000441f,0.0f, 0.01594f,0.0f,-0.000441f,0.0f,0.01934f};
					matrix::SquareMatrix<float, 3> _inertia_tensor(bldr_array_inertia);

					// matrix::Vector3f _alpha_ref_temp(0.0f, 0.0f, 0.0f);
					// _alpha_reference_body = _alpha_ref_temp;

					float _temp_scalar_1 = _alpha_reference_body(0)+la1/2.0f*(C_br(0,1)*_omega_tilde(1) + C_br(0,2)*_omega_tilde(2) - C_br(1,1)*_omega_tilde(0) - C_br(2,2)*_omega_tilde(0));
					float _temp_scalar_2 = _alpha_reference_body(1)-la2/2.0f*(C_br(0,0)*_omega_tilde(1) - C_br(1,0)*_omega_tilde(0) - C_br(1,2)*_omega_tilde(2) + C_br(2,2)*_omega_tilde(1));
					float _temp_scalar_3 = _alpha_reference_body(2)+la3/2.0f*(C_br(2,0)*_omega_tilde(0) - C_br(1,1)*_omega_tilde(2) - C_br(0,0)*_omega_tilde(2) + C_br(2,1)*_omega_tilde(1));
					matrix::Vector3f _temp_vector(_temp_scalar_1,_temp_scalar_2,_temp_scalar_3);
					matrix::Vector3f _tau_ff(0.0f,0.0f,0.0f);
					_tau_ff = _inertia_tensor*_temp_vector;

					float bldr_array_crossm[9] = {0.0f, -_omega_aux(2), _omega_aux(1), _omega_aux(2), 0.0f, -_omega_aux(0), -_omega_aux(1), _omega_aux(0),0.0f};
					matrix::SquareMatrix<float, 3> _crossm_omega_aux(bldr_array_crossm);
					matrix::Vector3f _tau_plus(0.0f,0.0f,0.0f);
					_tau_plus = _crossm_omega_aux*_inertia_tensor*_omega_estimate;

					_tau_1 = -0.7f*Kad1*(x_rate_body-_omega_reference_body(0))+1.2f*0.8f*Kap1*err_att_1+_tau_ff(0)+0.0f*_tau_plus(0);
					_tau_2 = -0.6f*Kad2*(y_rate_body-_omega_reference_body(1))+1.2f*0.8f*Kap2*err_att_2+_tau_ff(1)+0.0f*_tau_plus(1);
					_tau_3 = -0.8f*Kad3*(z_rate_body-_omega_reference_body(2))+1.2f*0.8f*Kap3*err_att_3+_tau_ff(2)+0.0f*_tau_plus(2);


				}
//
//
//
//
// /*............................................................................*/
				float Vs = 4.0f;
				if (_slipstream_approx < 1.0f)
				{
					float Vmin = 4.0f;

					float Vtot = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy + _local_pos.vz * _local_pos.vz);
					float Faero = (2)*(0.0157f*Vtot*Vtot - 0.0524f*Vtot + 0.5583f); // (1) should be replaced by Kaero
					matrix::Eulerf euler_angles(C_bi.transpose());
					float T2 = Faero + 0.45f*9.81f*sinf(euler_angles.theta());
					if(T2 < 0.0f){T2 = 0.0f;}
					float u = C_bi(0, 0) * _local_pos.vx + C_bi(0, 1) * _local_pos.vy + C_bi(0, 2) * _local_pos.vz;
					Vs = sqrt(u*u + (2.0f*T2)/(1.225f * 0.05067f));

					if(Vs < Vmin){
						Vs = Vmin;
					}
					// Vs = Vs/4.0f;
				}
				else{
					Vs = _slipstream_approx;
				}





				_slips_estimate = Vs;
				_AilDef=0.7f*_tau_1/(.5f*ro*powf(Vs,2.0f)*S_area*b_span*Cl_delta_a); //Aileron Deflection (deg)
				_ElevDef=0.8f*_tau_2/(.5f*ro*powf(Vs,2.0f)*S_area*c_bar*Cm_delta_e); //Elevator Deflection (deg)
				_RudDef=0.6f*_tau_3/(.5f*ro*powf(Vs,2.0f)*S_area*b_span*Cn_delta_r); //Rudder Deflection (deg)

}

void FixedwingAttitudeControl::JUAN_pose_initialize()
{
		// float _added_initial_distance = 0.0f;
		// _pos_x_initial = _pos_x_ref+_added_initial_distance*cosf(_heading_path);
		// _pos_y_initial = _pos_y_ref+_added_initial_distance*sinf(_heading_path);
		if (_control_mode_change_direction == 0) {
			float _added_initial_distance = 0.0f;
			_pos_x_initial = _pos_x_ref+_added_initial_distance*cosf(_heading_path);
			_pos_y_initial = _pos_y_ref+_added_initial_distance*sinf(_heading_path);
			_pos_z_initial = _pos_z_ref;
		}
		else if (_control_mode_change_direction == 1) {
			float _added_initial_distance = 0.0f;
			_err_vel_x_int = _pos_x_ref+_added_initial_distance*cosf(_heading_path);
			_err_vel_y_int = _pos_y_ref+_added_initial_distance*sinf(_heading_path);
			_err_vel_z_int = _pos_z_ref;
			_pos_x_initial = _pos_x_ref;
			_pos_y_initial = _pos_y_ref;
			_pos_z_initial = _pos_z_ref;

		}
		else if (_control_mode_change_direction == 2)
		{
			_pos_x_initial = _pos_x_ref - _err_vel_x_int;
			_pos_y_initial = _pos_y_ref - _err_vel_y_int;
			_pos_z_initial = _pos_z_ref - _err_vel_z_int;
		}
		else if(_control_mode_change_direction == 3)
		{
			_err_vel_x_int = _pos_x_ref - _pos_x_initial;
			_err_vel_y_int = _pos_y_ref - _pos_y_initial;
			_err_vel_z_int = _pos_z_ref - _pos_z_initial;
			_pos_x_initial = _pos_x_ref;
			_pos_y_initial = _pos_y_ref;
			_pos_z_initial = _pos_z_ref;
		}
		else if(_control_mode_change_direction == 4)
		{
			_pos_x_initial = _pos_x_ref;
			_pos_y_initial = _pos_y_ref;
			_pos_z_initial = _pos_z_ref;
		}

		_sigma_0 = _sigma;
		_initial_heading = _heading_path;

}

void FixedwingAttitudeControl::Unit_Speed_line(float Xi, float Ga)
{
 _pos_x_ref = (_sigma-_sigma_0)*cosf(Xi)*cosf(Ga)+_pos_x_initial;
 _pos_y_ref = (_sigma-_sigma_0)*sinf(Xi)*cosf(Ga)+_pos_y_initial;
 _pos_z_ref = -(_sigma-_sigma_0)*sinf(Ga)+_pos_z_initial;

 _Tv1 = cosf(Xi)*cosf(Ga);
 _Tv2 = sinf(Xi)*cosf(Ga);
 _Tv3 = -sinf(Ga);

 _heading_path = Xi;
}

void FixedwingAttitudeControl::Unit_Speed_helix(float _dir, float r, float c, float Xi)
{
		float x = r*sinf((_sigma-_sigma_0)/sqrtf(r*r+c*c));
    float y = -math::sign(_dir)*(r-r*cosf((_sigma-_sigma_0)/sqrtf(r*r+c*c)));
    float z = -c*(_sigma-_sigma_0)/sqrtf(r*r+c*c);

		float dx = r/sqrtf(r*r+c*c)*cosf((_sigma-_sigma_0)/sqrtf(r*r+c*c));
    float dy = -math::sign(_dir)*r/sqrtf(r*r+c*c)*sinf((_sigma-_sigma_0)/sqrtf(r*r+c*c));
    float dz = -c/sqrtf(r*r+c*c);

		_pos_x_ref = (cosf(Xi)*x-sinf(Xi)*y)+_pos_x_initial;
		_pos_y_ref = (sinf(Xi)*x+cosf(Xi)*y)+_pos_y_initial;
		_pos_z_ref = z+_pos_z_initial;

		_Tv1 = dx*cosf(Xi)-sinf(Xi)*dy;
		_Tv2 = dx*sinf(Xi)+cosf(Xi)*dy;
		_Tv3 = dz;

		_heading_path = atan2(dy,dx)+Xi;
}


void FixedwingAttitudeControl::JUAN_Path_F_Helix_Lines(float Vel, float a, float b)
{
	if (fabsf(a) < 0.2f)
	{
	 	float _ga = 60.0f*b*3.1415f/180.0f;
		Unit_Speed_line(_initial_heading,_ga);
	}
	else
	{
		float _rmin = 10.0f;
		float _rMax = 100.0f;
		float _k = (_rmin-_rMax)/0.8f;
		float _r0 = (5.0f*_rMax-_rmin)/4.0f;
    float _r = _k*fabsf(a)+_r0;
		float _dir = a/fabsf(a);
		float _ga = 60.0f*b*3.1415f/180.0f;
		float _c = _r*tan(_ga);
		Unit_Speed_helix(_dir,_r,_c,_initial_heading);
	}

	float _Hv1 = -_Tv2/sqrtf(_Tv1*_Tv1+_Tv2*_Tv2);
	float _Hv2 = _Tv1/sqrtf(_Tv1*_Tv1+_Tv2*_Tv2);
	float _Hv3 = 0.0f;

	float _Pv1 = -_Tv3*_Hv2;
	float _Pv2 = _Tv3*_Hv1;
	float _Pv3 = -_Tv2*_Hv1+_Tv1*_Hv2;


	float bldr_array_cpi[9] = {_Tv1,_Hv1,_Pv1,_Tv2,_Hv2,_Pv2,_Tv3,_Hv3,_Pv3};
	matrix::SquareMatrix<float, 3> Bldr_Matrix_cpi(bldr_array_cpi);
	matrix::Dcmf C_path (Bldr_Matrix_cpi); // DCM cast transposes?

	float _err_path_x = _pos_x_est - _pos_x_ref;
	float _err_path_y = _pos_y_est - _pos_y_ref;
	float _err_path_z = _pos_z_est - _pos_z_ref;


	matrix::Vector3f _Tangent_V(_Tv1,_Tv2,_Tv3);
	matrix::Vector3f _Horizontal_V(_Hv1,_Hv2,_Hv3);
	matrix::Vector3f _Projected_V(_Pv1,_Pv2,_Pv3);



	matrix::Vector3f _err_path(_err_path_x,_err_path_y,_err_path_z);
	matrix::Vector3f _err_path_GN = C_path*_err_path;

	// Approach direction
	float _LookA = 2.0f;

	matrix::Vector3f _LA_V = _LookA*_Tangent_V - _err_path_GN(1)*_Horizontal_V - _err_path_GN(2)*_Projected_V;

 _vel_x_ref = Vel*_LA_V(0)/sqrtf(_LA_V(0)*_LA_V(0)+_LA_V(1)*_LA_V(1)+_LA_V(2)*_LA_V(2));
 _vel_y_ref = Vel*_LA_V(1)/sqrtf(_LA_V(0)*_LA_V(0)+_LA_V(1)*_LA_V(1)+_LA_V(2)*_LA_V(2));
 _vel_z_ref = Vel*_LA_V(2)/sqrtf(_LA_V(0)*_LA_V(0)+_LA_V(1)*_LA_V(1)+_LA_V(2)*_LA_V(2));

 // matrix::Vector3f _vel_Inert(_vel_x_est,_vel_y_est,_vel_z_est); // Change THIS

 matrix::Vector3f _vel_B(Vel,0.0f,0.0f); // Change THIS
 matrix::Vector3f _vel_Inert = C_bi.transpose()*_vel_B;
 float _Kl = 4.0f;

 // matrix::Vector3f _temp_aux = _vel_Inert+_Kl*_err_path;

 float _aux1 = _vel_Inert(0)+_Kl*_err_path_x;
 float _aux2 = _vel_Inert(1)+_Kl*_err_path_y;
 float _aux3 = _vel_Inert(2)+_Kl*_err_path_z;


 _d_sigma =  _aux1*_Tv1 + _aux2*_Tv2+_aux3*_Tv3;

 if (_d_sigma < 0.0f) {
 	  _d_sigma = 0.0f;
 }


	// float _d_sigma = Vel;
	// _vel_x_ref = Vel*_Tv1;
	// _vel_y_ref = Vel*_Tv2;
	// _vel_z_ref = Vel*_Tv3;

	_sigma = _sigma + _d_sigma*_delta_time_attitude;


}


void FixedwingAttitudeControl::JUAN_Add_Roll()
{
	// Added roll gains
	float k_roll_p = 1.0f*1.0f*35.0f*0.005f*4.32f;
	float k_roll_y = 1.0f*1.0f*1.5f*0.7f*0.02f*0.4f;
	float max_roll = 30.0f;
	float k_roll_i = 0.0f*0.01f;
	float _heading_ref = atan2f(_vel_y_ref,_vel_x_ref);

	if (_heading_ref < -PI_f)
	{
			_heading_ref = _heading_ref + 2*PI_f;
	}
	else if (_heading_ref > PI_f)
	{
			_heading_ref = _heading_ref - 2*PI_f;
	}

	if (_velocity_control_flag == 0) {
		float _error_pos_x = _pos_x_ref-_pos_x_est;
		float _error_pos_y = _pos_y_ref-_pos_y_est;
		float _error_pos_z = _pos_z_ref-_pos_z_est;

		matrix::Vector3f error_inertial(_error_pos_x, _error_pos_y, _error_pos_z);
		matrix::Vector3f error_body = C_bi*error_inertial;
		// _lateral_error_roll = error_body(1);
		_lateral_error_roll = 0.0f;
	}
	else{
		if (_path_following_flag == 0) {
			_lateral_error_roll = 0.0f;
		}
		else{
			_lateral_error_roll = -0.25*1.0f*_error_crosstrack;
		}
	}


	float _heading_test = atan2f(_vel_y_est,_vel_x_est);

		if (_heading_test < -PI_f)
		{
				_heading_test = _heading_test + 2*PI_f;
		}
		else if (_heading_test > PI_f)
		{
				_heading_test = _heading_test - 2*PI_f;
		}

		float heading_aux = _heading_ref + atanf(k_roll_y*_lateral_error_roll);

		if (heading_aux < -PI_f)
		{
				heading_aux = heading_aux + 2*PI_f;
		}
		else if (heading_aux > PI_f)
		{
				heading_aux = heading_aux - 2*PI_f;
		}

		float heading_com = heading_aux-_heading_test;

		if (heading_com < -PI_f)
		{
				heading_com = heading_com + 2*PI_f;
		}
		else if (heading_com > PI_f)
		{
				heading_com = heading_com - 2*PI_f;
		}



		_error_heading_int = _error_heading_int + (heading_aux-_heading_test)*_delta_time_attitude;
		_roll_com = 1.0f*(0.8f*k_roll_p*heading_com+0.5f*k_roll_i*_error_heading_int);

		if (_roll_com >= 0.0f)
		{
			if (_roll_com > max_roll*PI_f/180)
			{
				_roll_com = max_roll*PI_f/180;
			}
		}
		else {
			if (_roll_com < -max_roll*PI_f/180)
			{
				_roll_com = -max_roll*PI_f/180;
			}
		}

		// barrel_roll = barrel_roll+5.0f*_delta_time_attitude;
		// _roll_com = barrel_roll;
		//
		// _roll_com = 3.1416f/2.0f;

		// _roll_com = 0.0f;


}

void FixedwingAttitudeControl::JUAN_provisional_path_following()
{
	float _Vel_com = 10.0f;
	float _k_h = 0.1f*1.0f;
	float _k_c = 1.0f*1.0f;
	float _k_s = 0.5f*0.1f*8.0f;
	float _d_a = 7.0f;

	Unit_Speed_helix(-1.0f, 10.0f, -2.0f, _initial_heading);
	// Unit_Speed_line(_initial_heading, 0.0f);

	float _Hv1 = -_Tv2/sqrtf(_Tv1*_Tv1+_Tv2*_Tv2);
	float _Hv2 = _Tv1/sqrtf(_Tv1*_Tv1+_Tv2*_Tv2);
	float _Hv3 = 0.0f;

	float _Pv1 = -_Tv3*_Hv2;
	float _Pv2 = _Tv3*_Hv1;
	float _Pv3 = -_Tv2*_Hv1+_Tv1*_Hv2;

	float norm_Pv = sqrtf(_Pv1*_Pv1+_Pv2*_Pv2+_Pv3*_Pv3);


	_Pv1 = _Pv1/norm_Pv;
	_Pv2 = _Pv2/norm_Pv;
	_Pv3 = _Pv3/norm_Pv;

	//
	// float bldr_array_cpi[9] = {_Tv1,_Hv1,_Pv1,_Tv2,_Hv2,_Pv2,_Tv3,_Hv3,_Pv3};
	// matrix::SquareMatrix<float, 3> Bldr_Matrix_cpi(bldr_array_cpi);
	// matrix::Dcmf C_path (Bldr_Matrix_cpi); // DCM cast transposes?

	float _err_path_x = _pos_x_est - _pos_x_ref;
	float _err_path_y = _pos_y_est - _pos_y_ref;
	float _err_path_z = _pos_z_est - _pos_z_ref;

	// matrix::Vector3f _err_path(_err_path_x,_err_path_y,_err_path_z);
	// matrix::Vector3f _err_path_GN = C_path*_err_path;
	//
	//
	// _error_onpath = _err_path_GN(0);
	// _error_crosstrack = _err_path_GN(1);
	// _error_altitude = _err_path_GN(2);

	_error_onpath = _err_path_x*_Tv1 + _err_path_y*_Tv2 + _err_path_z *_Tv3;
	_error_crosstrack = _err_path_x*_Hv1 + _err_path_y*_Hv2 + _err_path_z *_Hv3;
	_error_altitude = _err_path_x*_Pv1 + _err_path_y*_Pv2 + _err_path_z *_Pv3;


	float k_tst = _Vel_com/_d_a;

	float _La_1 = k_tst*(_d_a*_Tv1-_k_c*_error_crosstrack*_Hv1-_k_h*_error_altitude*_Pv1);
	float _La_2 = k_tst*(_d_a*_Tv2-_k_c*_error_crosstrack*_Hv2-_k_h*_error_altitude*_Pv2);
	float _La_3 = k_tst*(_d_a*_Tv3-_k_c*_error_crosstrack*_Hv3-_k_h*_error_altitude*_Pv3);

	_vel_x_ref = _La_1;
	_vel_y_ref = _La_2;
	_vel_z_ref = _La_3;

	 _d_sigma =  _Vel_com + _k_s*_error_onpath;

	 if (_d_sigma < 0.0f) {
	 	  _d_sigma = 0.0f;
	 }
	 _sigma = _sigma + _d_sigma*_delta_time_attitude;
	 }


void FixedwingAttitudeControl::JUAN_manual_PF_manager()
{

	//
	float _comm_V = _read_thrust_stick*10.0f;
	int _cast_comm_V = static_cast<int>(_comm_V);
	_Vel_PF = 3.5f+8.0f*(_cast_comm_V/10.0f);

	// _Vel_PF = 10.0f;


	// _Vel_PF = 0.0f+10.0f*_read_thrust_stick;

	if (_Vel_PF>0.0f) {
		_position_control_flag = 1;
		if (_path_following_flag == 0) {
			_control_mode_change_direction = 3;
			JUAN_pose_initialize();
		}
		_path_following_flag = 1;
		_velocity_control_flag = 1;
		_probe_var = 1.0f;
	}
	else
	{
		if (_path_following_flag == 1)
		{
			_control_mode_change_direction = 2;
			JUAN_pose_initialize();
		}
		_position_control_flag = 1;
		_path_following_flag = 0;
		_velocity_control_flag = 0;
		_probe_var = 0.0f;
	}

	/* Thesis path gains, do not delete!!!! */
	// float _k_h = 1.0f*0.1f*1.0f;
	// float _k_c = 1.0f*1.0f*1.0f;
	// float _k_s = 1.0f*0.5f*0.1f*8.0f;
	// float _d_a = 7.0f;

	float _k_h = 1.5f*0.1f*1.0f;
	float _k_c = 1.8f*1.0f*1.0f;
	float _k_s = 1.0f*0.5f*0.1f*8.0f;
	float _d_a = 7.0f;


	// float _k_h = 1.0f;
	// float _k_c = 2.0f;
	// float _k_s = 2.0f;
	// float _d_a = 5.0f;



	// if (_time_elapsed < 10.0f)
	// {
	// 		_a_PF = 0.0f;
	// }
	// else if(_time_elapsed < 25.0f)
	// {
	// 	_a_PF = -0.6f;
	// }
	// else{
	// 	_a_PF = 0.0f;
	// }
	float _comm_a = _read_roll_stick*5.0f;
	int _cast_comm_a = static_cast<int>(_comm_a);
	// _a_PF = 0.0f;
	_a_PF = -1.0f*_cast_comm_a/5.0f;

	float _comm_b = _read_pitch_stick*5.0f;
	int _cast_comm_b = static_cast<int>(_comm_b);
	// _b_PF = 0.0f;
	_b_PF = -1.0f*_cast_comm_b/5.0f;

	// _Vel_PF = 10.0f;

	// float _comm_r = _read_pitch_stick*5.0f;
	// int _cast_comm_r = static_cast<int>(_comm_r);
	// // _b_PF = 0.0f;
	// _r_PF = -1.0f*_cast_comm_r/5.0f;

	// if (_Vel_PF>5.0f)
	// {
			float _Vel_com = _Vel_PF;
			if (_a_PF != _a_PF_prev || _b_PF != _b_PF_prev)
			{
				_control_mode_change_direction = 4;
				JUAN_pose_initialize();
			}

			_a_PF_prev = _a_PF;
			_b_PF_prev = _b_PF;
			_Vel_PF_prev = _Vel_PF;


			if(fabsf(_a_PF) < 0.2f)
			{
				float _gamma;
				if(_b_PF<0.0f)
				{
					_gamma = 20.0f*_b_PF*3.1415f/180.0f;
				}
				else
				{
					_gamma = 50.0f*_b_PF*3.1415f/180.0f;
				}
				Unit_Speed_line(_initial_heading, _gamma);
			}

			else
			{
				float _rmin = 10.0f;
				float _rMax = 30.0f;
				float _k = (_rmin-_rMax)/0.8f;
				float _r0 = (5.0f*_rMax-_rmin)/4.0f;
				float _r = _k*fabsf(_a_PF)+_r0;
				float _dir = _a_PF/fabsf(_a_PF);
				float _ga;
				if (_b_PF<0.0f) {
					_ga = 20.0f*_b_PF*3.1415f/180.0f;
				}
				else
				{
					_ga = 50.0f*_b_PF*3.1415f/180.0f;
				}
				float _c = _r*tan(_ga);
				Unit_Speed_helix(_dir,_r,_c,_initial_heading);
			}


			float _Hv1 = -_Tv2/sqrtf(_Tv1*_Tv1+_Tv2*_Tv2);
			float _Hv2 = _Tv1/sqrtf(_Tv1*_Tv1+_Tv2*_Tv2);
			float _Hv3 = 0.0f;

			float _Pv1 = -_Tv3*_Hv2;
			float _Pv2 = _Tv3*_Hv1;
			float _Pv3 = -_Tv2*_Hv1+_Tv1*_Hv2;

			float norm_Pv = sqrtf(_Pv1*_Pv1+_Pv2*_Pv2+_Pv3*_Pv3);


			_Pv1 = _Pv1/norm_Pv;
			_Pv2 = _Pv2/norm_Pv;
			_Pv3 = _Pv3/norm_Pv;

			//
			// float bldr_array_cpi[9] = {_Tv1,_Hv1,_Pv1,_Tv2,_Hv2,_Pv2,_Tv3,_Hv3,_Pv3};
			// matrix::SquareMatrix<float, 3> Bldr_Matrix_cpi(bldr_array_cpi);
			// matrix::Dcmf C_path (Bldr_Matrix_cpi); // DCM cast transposes?

			float _err_path_x = _pos_x_est - _pos_x_ref;
			float _err_path_y = _pos_y_est - _pos_y_ref;
			float _err_path_z = _pos_z_est - _pos_z_ref;

			// matrix::Vector3f _err_path(_err_path_x,_err_path_y,_err_path_z);
			// matrix::Vector3f _err_path_GN = C_path*_err_path;
			//
			//
			// _error_onpath = _err_path_GN(0);
			// _error_crosstrack = _err_path_GN(1);
			// _error_altitude = _err_path_GN(2);

			_error_onpath = _err_path_x*_Tv1 + _err_path_y*_Tv2 + _err_path_z *_Tv3;
			_error_crosstrack = _err_path_x*_Hv1 + _err_path_y*_Hv2 + _err_path_z *_Hv3;
			_error_altitude = _err_path_x*_Pv1 + _err_path_y*_Pv2 + _err_path_z *_Pv3;


			float k_tst = _Vel_com/_d_a;

			float _La_1 = k_tst*(_d_a*_Tv1-_k_c*_error_crosstrack*_Hv1-_k_h*_error_altitude*_Pv1);
			float _La_2 = k_tst*(_d_a*_Tv2-_k_c*_error_crosstrack*_Hv2-_k_h*_error_altitude*_Pv2);
			float _La_3 = k_tst*(_d_a*_Tv3-_k_c*_error_crosstrack*_Hv3-_k_h*_error_altitude*_Pv3);

			_vel_x_ref = _La_1;
			_vel_y_ref = _La_2;
			_vel_z_ref = _La_3;

			 _d_sigma =  _Vel_com + _k_s*_error_onpath;

			 if (_d_sigma < 0.0f) {
			 	  _d_sigma = 0.0f;
			 }
			 _sigma = _sigma + _d_sigma*_delta_time_attitude;

			/* comment out from here to end */
		 // }
		 // else{
			//  float _V_fb = _b_PF*2.0f;
			//  float _V_lat = -_a_PF*2.0f;
		 //
			//  _vel_x_ref = _V_fb*cosf(_initial_heading)-_V_lat*sinf(_initial_heading);
			//  _vel_y_ref = _V_fb*sinf(_initial_heading)+_V_lat*cosf(_initial_heading);
			//  _vel_z_ref = 0.0f;
		 //
			//  _pos_x_ref = _vel_x_ref*_delta_time_attitude + _pos_x_initial;
			//  _pos_y_ref = _vel_y_ref*_delta_time_attitude + _pos_y_initial;
			//  _pos_z_ref = _pos_z_initial;
		 //
			//  _pos_x_initial = _pos_x_ref;
			//  _pos_y_initial = _pos_y_ref;
			//  _pos_z_initial = _pos_z_ref;
		 //
		 //
		 //
		 // }
	}

void FixedwingAttitudeControl::JUAN_loop_experiments()
{
	_PD_plus_flag = 0;
	_slipstream_approx = 5.0f;
	_attitude_man_duration = 2.0f;
	_attitude_error_innovation = 1;
	_attitude_maneuver = 2;
	if(flick_counter < 3)
	{
		_locked_heading = _initial_heading;
		_thrust_hard_value = 1.0f;
	}
	else if (flick_counter < 4)
	{
		_thrust_hard_value = 0.75f;
	}
	else if (flick_counter < 5)
	{
		_thrust_hard_value = 0.5f;
	}
	else
	{
		_locked_heading = _initial_heading;
		_thrust_hard_value = 1.0f;
	}

}


	void FixedwingAttitudeControl::JUAN_slanted_loop_experiments()
	{

		_slipstream_approx = 5.0f;
		_attitude_man_duration = 2.0f;
		_attitude_error_innovation = 1;
		_attitude_maneuver = 10;
		_thrust_hard_value = 1.0f;
		if(flick_counter < 3)
		{
			_locked_heading = _initial_heading;
			_PD_plus_flag = 0;
		}
		else if (flick_counter < 4)
		{
			_PD_plus_flag = 1;
		}
		else
		{
			_locked_heading = _initial_heading;
			_PD_plus_flag = 0;
		}
	}

 void FixedwingAttitudeControl::JUAN_ATA_experiments()
 {
	 _slipstream_approx = 5.0f;
	 _attitude_man_duration = 1.5f;
	 _attitude_error_innovation = 1;
	 _attitude_maneuver = 3;
	 _thrust_hard_value = 1.0f;
	 if(flick_counter < 3)
	 {
		 _locked_heading = _initial_heading;
		 _PD_plus_flag = 0;
	 }
	 else if (flick_counter < 4)
	 {
		 _PD_plus_flag = 1;
	 }
	 else
	 {
		 _locked_heading = _initial_heading;
		 _PD_plus_flag = 0;
	 }
 }
 void FixedwingAttitudeControl::JUAN_harrier_experiments()
 {
	 _slipstream_approx = 5.0f;
	 _attitude_man_duration = 1.5f;
	 _attitude_error_innovation = 1;
	 _attitude_maneuver = 8;

	 if(flick_counter < 3)
	 {
		 	_thrust_hard_value = 1.0f;
		 _locked_heading = _initial_heading;
		 _PD_plus_flag = 0;
	 }
	 else if (flick_counter < 4)
	 {
		  _thrust_hard_value = 1.0f;
		 _PD_plus_flag = 1;
	 }
	 else if (flick_counter < 4)
	 {
		 _PD_plus_flag = 0;
		 _thrust_hard_value = 0.75f;
	 }
	 else if (flick_counter < 5)
	 {
		 _PD_plus_flag = 1;
		 _thrust_hard_value = 0.75f;
	 }
	 else
	 {
		 _locked_heading = _initial_heading;
		 _PD_plus_flag = 0;
		 _thrust_hard_value = 1.0f;
	 }
 }

 void FixedwingAttitudeControl::JUAN_sudden_experiments()
 {
	_slipstream_approx = 5.0f;
	_attitude_maneuver = 7;
	_thrust_hard_value = 1.0f;
	_PD_plus_flag = 0;

	if(flick_counter < 3)
	{
		_attitude_error_innovation = 1;
		_locked_heading = _initial_heading;
	}
	else if (flick_counter < 4)
	{
		 _attitude_error_innovation = 0;
	}
	else if (flick_counter < 4)
	{
		_attitude_error_innovation = 2;
	}
	else
	{
		_locked_heading = _initial_heading;
		_attitude_error_innovation = 1;
	}
 }
