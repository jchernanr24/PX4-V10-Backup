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
using math::constrain;
using math::gradual;
using math::radians;

FixedwingAttitudeControl::FixedwingAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_fw) : ORB_ID(actuator_controls_0)),
	_attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// check if VTOL first
	if (vtol) {
		int32_t vt_type = -1;

		if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
			_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
		}
	}

	/* fetch initial parameter values */
	parameters_update();

	// set initial maximum body rate setpoints
	_roll_ctrl.set_max_rate(radians(_param_fw_acro_x_max.get()));
	_pitch_ctrl.set_max_rate_pos(radians(_param_fw_acro_y_max.get()));
	_pitch_ctrl.set_max_rate_neg(radians(_param_fw_acro_y_max.get()));
	_yaw_ctrl.set_max_rate(radians(_param_fw_acro_z_max.get()));
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
	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_param_fw_p_tc.get());
	_pitch_ctrl.set_k_p(_param_fw_pr_p.get());
	_pitch_ctrl.set_k_i(_param_fw_pr_i.get());
	_pitch_ctrl.set_k_ff(_param_fw_pr_ff.get());
	_pitch_ctrl.set_integrator_max(_param_fw_pr_imax.get());

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_param_fw_r_tc.get());
	_roll_ctrl.set_k_p(_param_fw_rr_p.get());
	_roll_ctrl.set_k_i(_param_fw_rr_i.get());
	_roll_ctrl.set_k_ff(_param_fw_rr_ff.get());
	_roll_ctrl.set_integrator_max(_param_fw_rr_imax.get());

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_param_fw_yr_p.get());
	_yaw_ctrl.set_k_i(_param_fw_yr_i.get());
	_yaw_ctrl.set_k_ff(_param_fw_yr_ff.get());
	_yaw_ctrl.set_integrator_max(_param_fw_yr_imax.get());

	/* wheel control parameters */
	_wheel_ctrl.set_k_p(_param_fw_wr_p.get());
	_wheel_ctrl.set_k_i(_param_fw_wr_i.get());
	_wheel_ctrl.set_k_ff(_param_fw_wr_ff.get());
	_wheel_ctrl.set_integrator_max(_param_fw_wr_imax.get());
	_wheel_ctrl.set_max_rate(radians(_param_fw_w_rmax.get()));

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
FixedwingAttitudeControl::wind_estimate_poll()
{
	_wind_sub.update(&_wind);
}

void
FixedwingAttitudeControl::vehicle_manual_poll()
{
	const bool is_tailsitter_transition = _is_tailsitter && _vehicle_status.in_transition_mode;
	const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	if (_vcontrol_mode.flag_control_manual_enabled && (!is_tailsitter_transition || is_fixed_wing)) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the _actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (!_vcontrol_mode.flag_control_climb_rate_enabled) {

				if (_vcontrol_mode.flag_control_attitude_enabled) {
					// STABILIZED mode generate the attitude setpoint from manual user inputs

					_att_sp.roll_body = _manual_control_setpoint.y * radians(_param_fw_man_r_max.get());

					_att_sp.pitch_body = -_manual_control_setpoint.x * radians(_param_fw_man_p_max.get())
							     + radians(_param_fw_psp_off.get());
					_att_sp.pitch_body = constrain(_att_sp.pitch_body,
								       -radians(_param_fw_man_p_max.get()), radians(_param_fw_man_p_max.get()));

					_att_sp.yaw_body = 0.0f;
					_att_sp.thrust_body[0] = math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f);

					Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					q.copyTo(_att_sp.q_d);

					_att_sp.timestamp = hrt_absolute_time();

					_attitude_sp_pub.publish(_att_sp);

				} else if (_vcontrol_mode.flag_control_rates_enabled &&
					   !_vcontrol_mode.flag_control_attitude_enabled) {

					// RATE mode we need to generate the rate setpoint from manual user inputs
					_rates_sp.timestamp = hrt_absolute_time();
					_rates_sp.roll = _manual_control_setpoint.y * radians(_param_fw_acro_x_max.get());
					_rates_sp.pitch = -_manual_control_setpoint.x * radians(_param_fw_acro_y_max.get());
					_rates_sp.yaw = _manual_control_setpoint.r * radians(_param_fw_acro_z_max.get());
					_rates_sp.thrust_body[0] = math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f);

					_rate_sp_pub.publish(_rates_sp);

				} else {
					/* manual/direct control */
					_actuators.control[actuator_controls_s::INDEX_ROLL] =
						_manual_control_setpoint.y * _param_fw_man_r_sc.get() + _param_trim_roll.get();
					_actuators.control[actuator_controls_s::INDEX_PITCH] =
						-_manual_control_setpoint.x * _param_fw_man_p_sc.get() + _param_trim_pitch.get();
					_actuators.control[actuator_controls_s::INDEX_YAW] =
						_manual_control_setpoint.r * _param_fw_man_y_sc.get() + _param_trim_yaw.get();
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f);
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
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().calibrated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if ((_param_fw_arsp_mode.get() == 0) && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().calibrated_airspeed_m_s);

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the minimum airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_min.get();
		}
	}

	/*
	 * For scaling our actuators using anything less than the min (close to stall)
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and its the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	const float airspeed_constrained = constrain(constrain(airspeed, _param_fw_airspd_min.get(),
					   _param_fw_airspd_max.get()), 0.1f, 1000.0f);

	_airspeed_scaling = (_param_fw_arsp_scale_en.get()) ? (_param_fw_airspd_trim.get() / airspeed_constrained) : 1.0f;

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

	// only run controller if attitude changed
	vehicle_attitude_s att;

	if (_att_sub.update(&att)) {

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		const float dt = math::constrain((att.timestamp - _last_run) * 1e-6f, 0.002f, 0.04f);
		_last_run = att.timestamp;

		/* get current rotation matrix and euler angles from control state quaternions */
		matrix::Dcmf R = matrix::Quatf(att.q);

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

		// vehicle status update must be before the vehicle_control_mode_poll(), otherwise rate sp are not published during whole transition
		_vehicle_status_sub.update(&_vehicle_status);

		vehicle_control_mode_poll();
		vehicle_manual_poll();
		vehicle_land_detected_poll();

		// the position controller will not emit attitude setpoints in some modes
		// we need to make sure that this flag is reset
		_att_sp.fw_control_yaw = _att_sp.fw_control_yaw && _vcontrol_mode.flag_control_auto_enabled;

		bool wheel_control = false;

		// TODO: manual wheel_control on ground?
		if (_param_fw_w_en.get() && _att_sp.fw_control_yaw) {
			wheel_control = true;
		}

		// lock integrator if no rate control enabled, or in RW mode (but not transitioning VTOL or tailsitter), or for long intervals (> 20 ms)
		/* JUAN modified integrator lock: will lock while in acro, unlock in stabilized*/
		bool lock_integrator = !(_vcontrol_mode.flag_control_rates_enabled && _vcontrol_mode.flag_control_attitude_enabled)
				       || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && ! _vehicle_status.in_transition_mode);

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}

		control_flaps(dt);

		/* decide if in stabilized or full manual control */
		if (_vcontrol_mode.flag_control_rates_enabled) {

			const float airspeed = get_airspeed_and_update_scaling();

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
			 * or a multicopter (but not transitioning VTOL or tailsitter)
			 */
			if (_landed
			    || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				&& !_vehicle_status.in_transition_mode && !_is_tailsitter)) {

				_roll_ctrl.reset_integrator();
				_pitch_ctrl.reset_integrator();
				_yaw_ctrl.reset_integrator();
				_wheel_ctrl.reset_integrator();
			}

			/* Prepare data for attitude controllers */
			ECL_ControlData control_input{};
			control_input.roll = euler_angles.phi();
			control_input.pitch = euler_angles.theta();
			control_input.yaw = euler_angles.psi();
			control_input.body_x_rate = rollspeed;
			control_input.body_y_rate = pitchspeed;
			control_input.body_z_rate = yawspeed;
			control_input.roll_setpoint = _att_sp.roll_body;
			control_input.pitch_setpoint = _att_sp.pitch_body;
			control_input.yaw_setpoint = _att_sp.yaw_body;
			control_input.airspeed_min = _param_fw_airspd_min.get();
			control_input.airspeed_max = _param_fw_airspd_max.get();
			control_input.airspeed = airspeed;
			control_input.scaler = _airspeed_scaling;
			control_input.lock_integrator = lock_integrator;

			if (wheel_control) {
				_local_pos_sub.update(&_local_pos);

				/* Use min airspeed to calculate ground speed scaling region.
				* Don't scale below gspd_scaling_trim
				*/
				float groundspeed = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);
				float gspd_scaling_trim = (_param_fw_airspd_min.get() * 0.6f);

				control_input.groundspeed = groundspeed;

				if (groundspeed > gspd_scaling_trim) {
					control_input.groundspeed_scaler = gspd_scaling_trim / groundspeed;

				} else {
					control_input.groundspeed_scaler = 1.0f;
				}
			}

			/* reset body angular rate limits on mode change */
			if ((_vcontrol_mode.flag_control_attitude_enabled != _flag_control_attitude_enabled_last) || params_updated) {
				if (_vcontrol_mode.flag_control_attitude_enabled
				    || _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					_roll_ctrl.set_max_rate(radians(_param_fw_r_rmax.get()));
					_pitch_ctrl.set_max_rate_pos(radians(_param_fw_p_rmax_pos.get()));
					_pitch_ctrl.set_max_rate_neg(radians(_param_fw_p_rmax_neg.get()));
					_yaw_ctrl.set_max_rate(radians(_param_fw_y_rmax.get()));

				} else {
					_roll_ctrl.set_max_rate(radians(_param_fw_acro_x_max.get()));
					_pitch_ctrl.set_max_rate_pos(radians(_param_fw_acro_y_max.get()));
					_pitch_ctrl.set_max_rate_neg(radians(_param_fw_acro_y_max.get()));
					_yaw_ctrl.set_max_rate(radians(_param_fw_acro_z_max.get()));
				}
			}

			_flag_control_attitude_enabled_last = _vcontrol_mode.flag_control_attitude_enabled;

			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			float trim_roll = _param_trim_roll.get();
			float trim_pitch = _param_trim_pitch.get();
			float trim_yaw = _param_trim_yaw.get();

			if (airspeed < _param_fw_airspd_trim.get()) {
				trim_roll += gradual(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(), _param_fw_dtrim_r_vmin.get(),
						     0.0f);
				trim_pitch += gradual(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(), _param_fw_dtrim_p_vmin.get(),
						      0.0f);
				trim_yaw += gradual(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(), _param_fw_dtrim_y_vmin.get(),
						    0.0f);

			} else {
				trim_roll += gradual(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						     _param_fw_dtrim_r_vmax.get());
				trim_pitch += gradual(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						      _param_fw_dtrim_p_vmax.get());
				trim_yaw += gradual(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						    _param_fw_dtrim_y_vmax.get());
			}

			/* add trim increment if flaps are deployed  */
			trim_roll += _flaps_applied * _param_fw_dtrim_r_flps.get();
			trim_pitch += _flaps_applied * _param_fw_dtrim_p_flps.get();

			/* Run attitude controllers */
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				if (PX4_ISFINITE(_att_sp.roll_body) && PX4_ISFINITE(_att_sp.pitch_body)) {
					_roll_ctrl.control_attitude(dt, control_input);
					_pitch_ctrl.control_attitude(dt, control_input);

					if (wheel_control) {
						_wheel_ctrl.control_attitude(dt, control_input);
						_yaw_ctrl.reset_integrator();

					} else {
						// runs last, because is depending on output of roll and pitch attitude
						_yaw_ctrl.control_attitude(dt, control_input);
						_wheel_ctrl.reset_integrator();
					}

					/* Update input data for rate controllers */
					control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
					control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
					control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

					/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
					float roll_u = _roll_ctrl.control_euler_rate(dt, control_input);
					_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;

					if (!PX4_ISFINITE(roll_u)) {
						_roll_ctrl.reset_integrator();
					}

					float pitch_u = _pitch_ctrl.control_euler_rate(dt, control_input);
					_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;

					if (!PX4_ISFINITE(pitch_u)) {
						_pitch_ctrl.reset_integrator();
					}

					float yaw_u = 0.0f;

					if (wheel_control) {
						yaw_u = _wheel_ctrl.control_bodyrate(dt, control_input);

					} else {
						yaw_u = _yaw_ctrl.control_euler_rate(dt, control_input);
					}

					_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

					/* add in manual rudder control in manual modes */
					if (_vcontrol_mode.flag_control_manual_enabled) {
						_actuators.control[actuator_controls_s::INDEX_YAW] += _manual_control_setpoint.r;
					}

					if (!PX4_ISFINITE(yaw_u)) {
						_yaw_ctrl.reset_integrator();
						_wheel_ctrl.reset_integrator();
					}

					/* throttle passed through if it is finite and if no engine failure was detected */
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(_att_sp.thrust_body[0])
							&& !_vehicle_status.engine_failure) ? _att_sp.thrust_body[0] : 0.0f;

					/* scale effort by battery status */
					if (_param_fw_bat_scale_en.get() &&
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

				//Initialize acro
				// JUAN: Preparing variables for custom mode transition
				_previous_yaw = euler_angles.psi();
				longTurn = true; //start off with a long turn
				// _initial_heading = _previous_yaw;
				_initial_heading = atan2f(_local_pos.vy, _local_pos.vx); //Use velocity direction instead
				_initial_vxy = sqrtf(_local_pos.vy * _local_pos.vy + _local_pos.vx * _local_pos.vx);
				_previous_time = hrt_absolute_time() / 1e6;
				_time_elapsed = 0.0f;
				_e_int_1 = 0.0f;
				_e_int_2 = 0.0f;
				_e_int_3 = 0.0f;

				_error_x_int = 0.0f;
				_error_y_int = 0.0f;
				_error_z_int = 0.0f;

				_control_operation_mode = 0;


				_yaw_test_profile = _previous_yaw;
				_pitch_test_profile = 0.0f;
				_roll_test_profile = 0.0f;

				_local_pos_sub.update(&_local_pos);
				float _added_initial_distance = 13.0f;
				// float _added_initial_distance = 10.0f;

				_pos_x_ref = _local_pos.x + _added_initial_distance * cosf(_initial_heading);
				_pos_y_ref = _local_pos.y + _added_initial_distance * sinf(_initial_heading);
				_pos_z_ref = _local_pos.z + 1.29f; //McFoamy is usually a few meters above the path

				_pos_x_initial = _pos_x_ref;
				_pos_y_initial = _pos_y_ref;
				_pos_z_initial = _pos_z_ref;

				/* ---- set man flag for new run ---- */
				completeFlag = false;
				t_last = 0; //Reset final time
				turnCount = 0;

				/* -- for resetting path -- */
				_pos_x_last_vtx = 0.0f;
				_pos_y_last_vtx = 0.0f;

				t_last_vertex = 0.0f;
				turnCount = 0;


				/* ---- reset feedforward stuff ---- */
				feedforward_flag = true;
				_acc_x_ref = 0.0f;
				_acc_y_ref = 0.0f;
				// _juan_att_var.feedforward_on = true;


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

				// /* JUAN attitude control starts here */

				// Time management
				float _time_loop_start = hrt_absolute_time();
				_time_loop_start = _time_loop_start; // gets rid of unused flag

				float _time_attitude_now = hrt_absolute_time() / 1e6;
				_delta_time_attitude = _time_attitude_now - _previous_time;
				_previous_time = _time_attitude_now;

				_time_elapsed = _time_elapsed + _delta_time_attitude;

				C_bi = R.transpose(); //Cbi
				matrix::Eulerf euler_now(R); //Euler

				float x_rate_body = angular_velocity.xyz[0]; //omega
				float y_rate_body = angular_velocity.xyz[1];
				float z_rate_body = angular_velocity.xyz[2];

				float _read_roll_stick = _manual_control_setpoint.y; //stick commands
				float _read_pitch_stick = _manual_control_setpoint.x;
				float _read_thrust_stick = _manual_control_setpoint.z;

				/*get rid of unused error while avoiding a comment out*/
				_read_roll_stick = _read_roll_stick;
				_read_pitch_stick = _read_pitch_stick;
				_read_thrust_stick = _read_thrust_stick;


				// // Manual attitude mode start

				// float _manual_roll = _read_roll_stick*0.785f;
				// float _manual_pitch = _read_pitch_stick*-0.785f;
				//
				//
				//
				//
				// // Velocity
				// matrix::Vector3f _inertial_velocity(_global_pos.vel_n, _global_pos.vel_e, _global_pos.vel_d);
				// matrix::Vector3f _body_velocity = C_bi*_inertial_velocity;
				// float _vel_ground = sqrtf(_body_velocity(0)*_body_velocity(0)+_body_velocity(1)+_body_velocity(1));
				//
				//
				//
				// if (_vel_ground <5.0f){
				// 	_ground_velocity_corrected = 5.0f;
				// }
				// else {
				// 	_ground_velocity_corrected = _vel_ground;
				// }
				// // float _vel_ground = 5.0f;
				//
				// // float _vel_ground = 10.0f;
				//
				// // JUAN note: tangent can cause issues if roll is allowed to go to 90 degrees!
				// float _heading_rate_coordinated = 1.5f*9.81f/_ground_velocity_corrected * tanf(_manual_roll);
				//
				// float _manual_yaw = _delta_time_attitude*_heading_rate_coordinated+_previous_yaw;
				// bool _manual_yaw_check = PX4_ISFINITE(_manual_yaw);
				//
				// if (_manual_yaw_check)
				// {
				// 	_previous_yaw = _manual_yaw;
				// }
				// else{
				// 	_manual_yaw = _previous_yaw;
				// }
				// _roll_rate_reference = 0.0f;
				// _pitch_rate_reference = 0.0f;
				// _yaw_rate_reference = _heading_rate_coordinated;


				// Manual attitude end


				/* ........................ Loop profile ...........................*/
				if (_time_elapsed < 1.0f) { //do nothing for a second
					_pitch_test_profile = 0.0f;
					_pitch_rate_reference = 0.0f;

				} else if (_time_elapsed < 3.0f) { //constant pitch rate
					_pitch_rate_reference = 3.1416f;
					_pitch_test_profile = _pitch_test_profile + _pitch_rate_reference * _delta_time_attitude;

				} else { //do nothing
					_pitch_rate_reference = 0.0f;
					_pitch_test_profile = 0.0f;
				}

				_yaw_rate_reference = 0.0f;
				_roll_rate_reference = 0.0f;

				float _manual_yaw = _yaw_test_profile;
				float _manual_roll = _roll_test_profile;
				float _manual_pitch = _pitch_test_profile;
				float _heading_rate_coordinated = -1.0f;
				_ground_velocity_corrected = -1.0f;
				/*.................. End Loop profile ..............................*/

				/*..................  ATA profile   ................................*/

				// if (_time_elapsed < 1.0f){
				// 	_pitch_rate_reference = 0.0f;
				//   _roll_rate_reference = 0.0f;
				// 	_pitch_test_profile = 0.0f;
				// 	_roll_test_profile = 0.0f;
				// }
				// else if (_time_elapsed <2.0f){
				// 	_pitch_rate_reference = 3.1416f/2.0f;
				// 	_pitch_test_profile = _pitch_test_profile + _pitch_rate_reference*_delta_time_attitude;
				//  _roll_rate_reference = 0.0f;
				// 	_roll_test_profile = 0.0f;
				// }
				// else if(_time_elapsed <3.0f){
				// 	_pitch_rate_reference = 3.1416f/2.0f;
				// 	_roll_rate_reference = 3.1416f;
				// 	_pitch_test_profile = _pitch_test_profile + _pitch_rate_reference*_delta_time_attitude;
				// 	_roll_test_profile = _roll_test_profile + _roll_rate_reference*_delta_time_attitude;
				// }
				// else {
				//  _pitch_rate_reference = 0.0f;
				//  _roll_rate_reference = 0.0f;
				// 	_pitch_test_profile = 3.1416f;
				// 	_roll_test_profile = 3.1415f;
				// 	// _yaw_test_profile = _previous_yaw+3.1416f;
				// }
				// _yaw_rate_reference = 0.0f;
				//
				// float _manual_yaw = _yaw_test_profile;
				// float _manual_roll = _roll_test_profile;
				// float _manual_pitch = _pitch_test_profile;
				// float _heading_rate_coordinated = -1.0f;
				// float _ground_velocity_corrected = -1.0f;

				/*................... End ATA profile...............................*/

				/*..DCMs for principal rotations (stabilized attitude control only).*/
				// _manual_yaw = 0.0f;
				// _manual_yaw = euler_now.psi();


				float bldr_array_Cyaw[9] = {cosf(_manual_yaw), sinf(_manual_yaw), 0.0f, -sinf(_manual_yaw), cosf(_manual_yaw), 0.0f, 0.0f, 0.0f, 1.0f};
				matrix::SquareMatrix<float, 3> Bldr_Matrix_Cyaw(bldr_array_Cyaw);
				matrix::Dcmf C_yaw(Bldr_Matrix_Cyaw);

				float bldr_array_Cpitch[9] = {cosf(_manual_pitch), 0.0f, -sinf(_manual_pitch), 0.0f, 1.0f, 0.0f, sinf(_manual_pitch), 0.0f, cosf(_manual_pitch)};
				matrix::SquareMatrix<float, 3> Bldr_Matrix_Cpitch(bldr_array_Cpitch);
				matrix::Dcmf C_pitch(Bldr_Matrix_Cpitch);

				float bldr_array_Croll[9] = {1.0f, 0.0f, 0.0f, 0.0f, cosf(_manual_roll), sinf(_manual_roll), 0.0f, -sinf(_manual_roll), cosf(_manual_roll)};
				matrix::SquareMatrix<float, 3> Bldr_Matrix_Croll(bldr_array_Croll);
				matrix::Dcmf C_roll(Bldr_Matrix_Croll);

				// float bldr_array_Ctest[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
				// matrix::SquareMatrix<float, 3> Bldr_Matrix_Ctest(bldr_array_Ctest);
				// matrix::Dcmf C_test(Bldr_Matrix_Ctest);

				matrix::Dcmf C_manual = C_roll * C_pitch * C_yaw;
				/*..................................................................*/
				// matrix::Dcmf C_manual = C_roll*C_pitch*C_yaw;
				// matrix::Dcmf C_manual = C_roll*C_pitch;

				/*... DCMs for control system ......................................*/
				_JUAN_flight_mode = 1;

				if (_JUAN_flight_mode < 1) { //if in juan's flight mode
					C_ri = C_manual; //set reference
					matrix::Dcmf C_riT = C_ri.transpose();
					matrix::Dcmf C_br_alt = C_bi * C_riT; //error dcm

					/*..................................................................*/

					/*.............  Euler rate to angular velocicty conversion ........*/
					matrix::Vector3f _reference_euler_rate(_roll_rate_reference, _pitch_rate_reference, _yaw_rate_reference);
					float bldr_array_e2w[9] = {1.0f, 0.0f, -sinf(_pitch_test_profile), 0.0f, cosf(_roll_test_profile), sinf(_roll_test_profile) *cosf(_pitch_test_profile), 0.0f, -sinf(_roll_test_profile), cosf(_roll_test_profile) *cosf(_pitch_test_profile)};
					matrix::SquareMatrix<float, 3> Bldr_Matrix_Ce2w(bldr_array_e2w);
					matrix::Dcmf C_e2w(Bldr_Matrix_Ce2w);
					matrix::Vector3f _omega_reference = C_e2w * _reference_euler_rate; // reference angular velocity in reference coord
					_omega_reference_body =  C_br_alt * _omega_reference; // reference angular velocity in body coord
					/*..................................................................*/
					_throttle_out = _manual_control_setpoint.z;

				} else {
					JUAN_position_control();
					C_ri = C_ri_pos.transpose();

					_juan_att_var.c_ri_pre_ff[0] =  C_ri(0, 0);
					_juan_att_var.c_ri_pre_ff[1] =  C_ri(0, 1);
					_juan_att_var.c_ri_pre_ff[2] =  C_ri(0, 2);
					_juan_att_var.c_ri_pre_ff[3] =  C_ri(1, 0);
					_juan_att_var.c_ri_pre_ff[4] =  C_ri(1, 1);
					_juan_att_var.c_ri_pre_ff[5] =  C_ri(1, 2);
					_juan_att_var.c_ri_pre_ff[6] =  C_ri(2, 0);
					_juan_att_var.c_ri_pre_ff[7] =  C_ri(2, 1);
					_juan_att_var.c_ri_pre_ff[8] =  C_ri(2, 2);

					if (feedforward_flag) {
						wind_estimate_poll(); //update estimate
						wind_ff_rot_update(); //update R_wind
						// matrix::Dcmf temp_C_ri = C_ri * R_wind; //avoid any weirdness like in Eigen
						// C_ri = temp_C_ri;
						C_ri = C_ri * R_wind;
					}

					_juan_att_var.feedforward_on = feedforward_flag;
					matrix::Vector3f _omega_ref_temp(0.0f, 0.0f, 0.0f); // change for command filter
					_omega_reference_body = _omega_ref_temp;


					// //Thrust mappings





					float Jar = 0.5f;

					/*..........Proper advance ratio calculation........................*/
					// float _vel_x_est = _local_pos.vx;
					// float _vel_y_est = _local_pos.vy;
					// float _vel_z_est = _local_pos.vz;
					// if (_previous_rpm < 1000)
					// {
					// 	_advance_ratio = 0.5f;
					// }
					// else
					// {
					// 	float _norm_vel =  sqrtf(_local_pos.vx*_local_pos.vx+_local_pos.vy*_local_pos.vy+_local_pos.vz*_local_pos.vz);
					// 	_advance_ratio = _norm_vel/(0.254f*(_previous_rpm/60.0f));
					// }
					// float Jar = saturate(_advance_ratio,0.0f,0.5f);
					/*..................................................................*/


					_global_jar = Jar;

					float kt = (-1.43909969f * Jar * Jar - 2.21240323f * Jar + 2.24512051f) * powf(10.0f, -7.0f);
					float omega_t = sqrtf(ThrustN / kt);
					_previous_rpm = omega_t;
					float thrust_PWM = saturate(1.6572f * powf(10.0f, -5.0f) * powf(omega_t, 2.0f) + .0166f * omega_t + 1121.8f, 1000.0f,
								    2000.0f);
					_throttle_out = (thrust_PWM - 1000.0f) / 1000.0f;
					// _throttle_out = _manual_control_setpoint.z;
				}



				matrix::Dcmf C_ir = C_ri.transpose();
				matrix::Dcmf C_br = C_bi * C_ir;

				/*.......SO(3) error calculation (and definition)...................*/
				// float att_err_modifier = 1.0f; // This is the standard
				float att_err_function = 0.0f;

				// float trace_C = C_br(0,0) + C_br(1,1) + C_br(2,2);
				// float err_att_aux = 1+trace_C;
				// if (err_att_aux < 0.01)
				// {
				// 	att_err_modifier = 1.0f;
				// }
				// else
				// {
				// 	// att_err_modifier = 2.0f/sqrtf(1+trace_C); //t. lee
				//  // float att_err_function = 1.0f;
				// 	att_err_modifier = 4.0f/(1+trace_C); //forbes
				// float att_err_function = 2.0f;
				// }

				//C_br = C_br.transpose();

				// float _a_b_1 = (C_br(1,2)-C_br(2,1));
				// float _a_b_2 = (C_br(0,2)-C_br(2,0));
				// float _a_b_3 = (C_br(0,1)-C_br(1,0));
				// matrix::Vector3f _a_b(_a_b_1,_a_b_2,_a_b_3);
				// matrix::Vector3f _a_i = C_ri.transpose()*_a_b;
				// matrix::Vector3f _a_i_bar(_a_i(0),_a_i(1),0.0f);
				// matrix::Vector3f _a_b_bar = C_ri*_a_i_bar;
				//
				// float err_att_1 = att_err_modifier*-0.5f*_a_b_bar(0);
				// float err_att_2 = att_err_modifier*0.5f*_a_b_bar(1);
				// float err_att_3 = att_err_modifier*-0.5f*_a_b_bar(2);


				float err_att_1 = -0.5f * (C_br(1, 2) - C_br(2, 1));
				float err_att_2 = 0.5f * (C_br(0, 2) - C_br(2, 0));
				float err_att_3 = -0.5f * (C_br(0, 1) - C_br(1, 0));
				/*..................................................................*/

				/* Constants used by control system ................................*/
				float S_area = 0.14274f; //Wing Area (m^2), yak54 = 0.14865
				float b_span = 0.864f; //Wing Span (m), yak54 = .82
				float c_bar = 0.21f; //Mean Aerodynamic Chord (m), yak54 =.2107
				float Cl_delta_a = -0.0006777f; //Aileron Control Derivative Coefficient (/deg)
				float Cm_delta_e = -0.0117747f; //Elevator Control Derivative Coefficient (/deg)
				float Cn_delta_r = -0.0035663f; //Rudder Control Derivative Coefficient (/deg)
				float ro = 1.225f;

				// float AilDef_max = 52.0f; //52.0fMaximum Aileron Deflection (deg)
				// float ElevDef_max = 59.0f; //35.0fMaximum Elevator Deflection (deg)
				// float RudDef_max = 49.0f; //56.0fMaximum Rudder Deflection (deg)
				// float omega_t_max = 6710.0f; //Maximimum Thrust (RPM)
				// float omega_t_min = 1716.0f; //Minimum Thrust (RPM)
				/*..................................................................*/

				/*................Attitude controller gains.........................*/
				// float Kad1 = 0.8f * 0.00706f;
				// float Kad2 = 0.5f * 0.07576f;
				// float Kad3 = 0.7f * 0.07736f;

				// float Kap1 = 1.0f * 0.1656f;
				// float Kap2 = 1.0f * 1.022f;
				// float Kap3 = 1.0f * 0.6776f;

				// float Kai1 = 0.0f * 0.8f * 0.1656f;
				// float Kai2 = 0.0f * 0.8f * 1.022f;
				// float Kai3 = 0.0f * 0.8f * 0.6776f;
				/*..................................................................*/
				/*................Attitude controller gains SITL!.........................*/
				float Kad1 = 0.0706f / (0.7f);
				float Kad2 = 0.6376f / (0.8f);
				float Kad3 = 0.7736f / (0.8f);

				float Kap1 = 0.7099f / (0.8f);
				float Kap2 = 0.35f * 25.5359f / (1.0f);
				float Kap3 = 0.35f * 38.7187f / (0.8f);

				float Kai1 = 0.0f * 0.8f * 0.1656f;
				float Kai2 = 0.0f * 0.8f * 1.022f;
				float Kai3 = 0.0f * 0.8f * 0.6776f;
				/*..................................................................*/

				/* Integral errors */
				_e_int_1 = _e_int_1 + _delta_time_attitude * _e_int_1;
				_e_int_2 = _e_int_2 + _delta_time_attitude * _e_int_2;
				_e_int_3 = _e_int_3 + _delta_time_attitude * _e_int_3;

				/*..................................................................*/


				/*.... SO(3) attitude control law ..................................*/

				// float tau_1 = -0.7f*Kad1*x_rate_body+0.8f*Kap1*err_att_1;
				// float tau_2 = -0.8f*Kad2*y_rate_body+1.0f*Kap2*err_att_2;
				// float tau_3 = -0.8f*Kad3*z_rate_body+0.8f*Kap3*err_att_3;

				// float tau_1 = -0.7f*Kad1*(x_rate_body-_omega_reference_body(0))+0.8f*Kap1*err_att_1;
				// float tau_2 = -0.8f*Kad2*(y_rate_body-_omega_reference_body(1))+0.8f*Kap2*err_att_2;
				// float tau_3 = -0.8f*Kad3*(z_rate_body-_omega_reference_body(2))+0.8f*Kap3*err_att_3;


				// float tau_1 = -0.7f*Kad1*x_rate_body+0.8f*Kap1*err_att_1+Kai1*_e_int_1;
				// float tau_2 = -0.8f*Kad2*y_rate_body+1.0f*Kap2*err_att_2+Kai2*_e_int_2;
				// float tau_3 = -0.8f*Kad3*z_rate_body+0.8f*Kap3*err_att_3+Kai3*_e_int_3;

				/*................. Final one!!!! ...................................*/
				float tau_1 = -0.7f * Kad1 * (x_rate_body - _omega_reference_body(0)) + 0.8f * Kap1 * err_att_1 + Kai1 * _e_int_1;
				float tau_2 = -0.8f * Kad2 * (y_rate_body - _omega_reference_body(1)) + 1.0f * Kap2 * err_att_2 + Kai2 * _e_int_2;
				float tau_3 = -0.8f * Kad3 * (z_rate_body - _omega_reference_body(2)) + 0.8f * Kap3 * err_att_3 + Kai3 * _e_int_3;


				/*.........Alternative Tracking control law ..................................*/
//
// 					/*.......Tracking law attitude controller gains.....................*/
// 					float la1 = 2.0f;
// 					float la2 = 2.0f;
// 					float la3 = 2.0f;
//
// 					float Kapp1 = Kap1 - la1*Kad1;
// 					float Kapp2 = Kap2 - la2*Kad2;
// 					float Kapp3 = Kap3 - la3*Kad3;
//
// 					/*.......Angular velocity error and composite error.................*/
// 					matrix::Vector3f _error_attitude(err_att_1,err_att_2,err_att_3);
// 					matrix::Vector3f _omega_estimate(x_rate_body,y_rate_body,z_rate_body);
// 					matrix::Vector3f _omega_tilde(0.0f,0.0f,0.0f);
// 					_omega_tilde = _omega_estimate - _omega_reference_body;
// 					matrix::Vector3f _sigma_error(_omega_tilde(0)-la1*err_att_1,_omega_tilde(1)-la2*err_att_2,_omega_tilde(2)-la3*err_att_3);
// 					matrix::Vector3f _omega_aux(_omega_reference_body(0)+la1*err_att_1,_omega_reference_body(1)+la2*err_att_2,_omega_reference_body(2)-la3*err_att_3);
//
//
// 					/*.....Control law..................................................*/
// 					float bldr_array_inertia[9] = {0.003922f, 0.0f, -0.000441f,0.0f, 0.01594f,0.0f,-0.000441f,0.0f,0.01934f};
// 					matrix::SquareMatrix<float, 3> _inertia_tensor(bldr_array_inertia);
//
// 					matrix::Vector3f _alpha_ref_temp(0.0f, 0.0f, 0.0f);
// 					_alpha_reference_body = _alpha_ref_temp;
//
// 					float _temp_scalar_1 = _alpha_reference_body(0)+la1/2.0f*(C_br(0,1)*_omega_tilde(1) + C_br(0,2)*_omega_tilde(2) - C_br(1,1)*_omega_tilde(0) - C_br(2,2)*_omega_tilde(0));
// 					float _temp_scalar_2 = _alpha_reference_body(1)-la2/2.0f*(C_br(0,0)*_omega_tilde(1) - C_br(1,0)*_omega_tilde(0) - C_br(1,2)*_omega_tilde(2) + C_br(2,2)*_omega_tilde(1));
// 					float _temp_scalar_3 = _alpha_reference_body(2)+la3/2.0f*(C_br(2,0)*_omega_tilde(0) - C_br(1,1)*_omega_tilde(2) - C_br(0,0)*_omega_tilde(2) + C_br(2,1)*_omega_tilde(1));
// 					matrix::Vector3f _temp_vector(_temp_scalar_1,_temp_scalar_2,_temp_scalar_3);
// 					matrix::Vector3f _tau_ff(0.0f,0.0f,0.0f);
// 					_tau_ff = _inertia_tensor*_temp_vector;
//
// 					float bldr_array_crossm[9] = {0.0f, -_omega_aux(2), _omega_aux(1), _omega_aux(2), 0.0f, -_omega_aux(0), -_omega_aux(1), _omega_aux(0),0.0f};
// 					matrix::SquareMatrix<float, 3> _crossm_omega_aux(bldr_array_crossm);
// 					matrix::Vector3f _tau_plus(0.0f,0.0f,0.0f);
// 					_tau_plus = _crossm_omega_aux*_inertia_tensor*_omega_estimate;
//
// 					float tau_1 = -0.7f*Kad1*(x_rate_body-_omega_reference_body(0))+0.8f*Kap1*err_att_1+_tau_ff(0)+_tau_plus(0);
// 					float tau_2 = -0.8f*Kad2*(y_rate_body-_omega_reference_body(1))+1.0f*Kap2*err_att_2+_tau_ff(1)+_tau_plus(1);
// 					float tau_3 = -0.8f*Kad3*(z_rate_body-_omega_reference_body(2))+0.8f*Kap3*err_att_3+_tau_ff(2)+_tau_plus(2);
//
//
//
//
// /*............................................................................*/


				/* .........................Control allocation......................*/
				// float airspeed2 = get_airspeed_and_update_scaling();
				// float Vs = airspeed2;
				float Vs = 6.0f;

				float AilDef = 0.7f * tau_1 / (.5f * ro * powf(Vs, 2.0f) * S_area * b_span * Cl_delta_a); //Aileron Deflection (deg)
				float ElevDef = 0.8f * tau_2 / (.5f * ro * powf(Vs, 2.0f) * S_area * c_bar * Cm_delta_e); //Elevator Deflection (deg)
				float RudDef = 0.6f * tau_3 / (.5f * ro * powf(Vs, 2.0f) * S_area * b_span * Cn_delta_r); //Rudder Deflection (deg)

				float outputs_ail = 0.0000016235f * powf(AilDef, 3.0f) - 0.0000009861f * powf(AilDef, 2.0f) + 0.0145866432f * AilDef;
				float outputs_ele = 0.0000008317f * powf(ElevDef, 3.0f) + 0.0000409759f * powf(ElevDef, 2.0f) + 0.01396963f * ElevDef;
				float outputs_rud = 0.0000007988f * powf(RudDef, 3.0f) + 0.0000092020f * powf(RudDef, 2.0f) + 0.0187045418f * RudDef;




				/*..................................................................*/

				/*........................... PX4 actuator mappings.................*/
				_actuators.control[actuator_controls_s::INDEX_ROLL] = -outputs_ail;
				_actuators.control[actuator_controls_s::INDEX_PITCH] = -outputs_ele;
				_actuators.control[actuator_controls_s::INDEX_YAW] = -outputs_rud;
				// Special command
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _throttle_out;
				/*..................................................................*/

				/*...................Custom logging ................................*/
				_juan_att_var.timestamp = hrt_absolute_time();


				// _juan_att_var.test_variable = _delta_time_attitude;
				//
				_juan_att_var.tau_ref[0] = tau_1;
				_juan_att_var.tau_ref[1] = tau_2;
				_juan_att_var.tau_ref[2] = tau_3;

				_juan_att_var.error_so3[0] = err_att_1;
				_juan_att_var.error_so3[1] = err_att_2;
				_juan_att_var.error_so3[2] = err_att_3;


				// _juan_att_var.ctr_defl[0] = AilDef;
				// _juan_att_var.ctr_defl[1] = ElevDef;
				// _juan_att_var.ctr_defl[2] = RudDef;

				// _juan_att_var.pwm_out_ctr[0] = outputs_ail;
				// _juan_att_var.pwm_out_ctr[1] = outputs_ele;
				// _juan_att_var.pwm_out_ctr[2] = outputs_rud;




				// matrix::Eulerf euler_now(C_bi.transpose());
				_juan_att_var.yaw_measured = euler_now.psi();
				_juan_att_var.pitch_measured = euler_now.theta();
				_juan_att_var.roll_measured = euler_now.phi();


				matrix::Eulerf euler_ref(C_ir);
				_juan_att_var.yaw_reference = euler_ref.psi();
				_juan_att_var.pitch_reference = euler_ref.theta();
				_juan_att_var.roll_reference = euler_ref.phi();


				_juan_att_var.pilot_roll_com = _manual_roll;
				_juan_att_var.pilot_pitch_com = _manual_pitch;
				_juan_att_var.pilot_yaw_com = _manual_yaw;
				_juan_att_var.ground_speed_attitude = _ground_velocity_corrected;
				_juan_att_var.coordinate_yaw_rate = _heading_rate_coordinated;
				_juan_att_var.attitude_error_function = att_err_function;


				_juan_att_var.omega_ref[0] = _omega_reference_body(0);
				_juan_att_var.omega_ref[1] = _omega_reference_body(1);
				_juan_att_var.omega_ref[2] = _omega_reference_body(2);

				_juan_att_var.omega_bod[0] = x_rate_body;
				_juan_att_var.omega_bod[1] = y_rate_body;
				_juan_att_var.omega_bod[2] = z_rate_body;

				_juan_att_var.cbi_rows[0] = C_bi(0, 0);
				_juan_att_var.cbi_rows[1] = C_bi(0, 1);
				_juan_att_var.cbi_rows[2] = C_bi(0, 2);
				_juan_att_var.cbi_rows[3] = C_bi(1, 0);
				_juan_att_var.cbi_rows[4] = C_bi(1, 1);
				_juan_att_var.cbi_rows[5] = C_bi(1, 2);
				_juan_att_var.cbi_rows[6] = C_bi(2, 0);
				_juan_att_var.cbi_rows[7] = C_bi(2, 1);
				_juan_att_var.cbi_rows[8] = C_bi(2, 2);

				_juan_att_var.cri_rows[0] = C_ri(0, 0);
				_juan_att_var.cri_rows[1] = C_ri(0, 1);
				_juan_att_var.cri_rows[2] = C_ri(0, 2);
				_juan_att_var.cri_rows[3] = C_ri(1, 0);
				_juan_att_var.cri_rows[4] = C_ri(1, 1);
				_juan_att_var.cri_rows[5] = C_ri(1, 2);
				_juan_att_var.cri_rows[6] = C_ri(2, 0);
				_juan_att_var.cri_rows[7] = C_ri(2, 1);
				_juan_att_var.cri_rows[8] = C_ri(2, 2);

				_airspeed_sub.update();
				float _airsp_indi_logged = _airspeed_sub.get().indicated_airspeed_m_s;
				float _airsp_true_logged = _airspeed_sub.get().true_airspeed_m_s;

				_juan_att_var.true_airspeed = _airsp_true_logged;
				_juan_att_var.indicated_airspeed = _airsp_indi_logged;



				_juan_att_var.test_variable = 12.0f;


				// matrix::Eulerf euler_ref(C_ri.transpose());
				// _juan_att_var.yaw_reference = euler_ref.psi();
				// _juan_att_var.pitch_reference = euler_ref.theta();
				// _juan_att_var.roll_reference = euler_ref.phi();


				// float _time_loop_end = hrt_absolute_time();
				// float _delta_run_time = _time_loop_end-_time_loop_start;
				//
				// _juan_att_var.test_variable = _delta_run_time;
				//


				// JUAN_reference_generator(1);
				if (_JUAN_flight_mode > 0) {
					_juan_att_var.reference_position_x = _pos_x_ref;
					_juan_att_var.reference_position_y = _pos_y_ref;
					_juan_att_var.reference_position_z = _pos_z_ref;

					_juan_att_var.reference_velocity_x = _vel_x_ref;
					_juan_att_var.reference_velocity_y = _vel_y_ref;
					_juan_att_var.reference_velocity_z = _vel_z_ref;

					// _local_pos_sub.update(&_local_pos);
					float _pos_x_est = _local_pos.x;
					float _pos_y_est = _local_pos.y;
					float _pos_z_est = _local_pos.z;

					//
					_juan_att_var.estimated_position_x = _pos_x_est;
					_juan_att_var.estimated_position_y = _pos_y_est;
					_juan_att_var.estimated_position_z = _pos_z_est;

					float _vel_x_log = _local_pos.vx;
					float _vel_y_log = _local_pos.vy;
					float _vel_z_log = _local_pos.vz;

					_juan_att_var.estimated_velocity_x = _vel_x_log;
					_juan_att_var.estimated_velocity_y = _vel_y_log;
					_juan_att_var.estimated_velocity_z = _vel_z_log;


					_juan_att_var.control_status = _control_operation_mode;
					_juan_att_var.j_advance = _global_jar;
					_juan_att_var.omega_rpm = _previous_rpm;
				}



				_juan_attitude_variables_pub.publish(_juan_att_var);
				/*..................................................................*/

				/*............... JUAN attitude control ends here.................... */
			}

			rate_ctrl_status_s rate_ctrl_status{};
			rate_ctrl_status.timestamp = hrt_absolute_time();
			rate_ctrl_status.rollspeed_integ = _roll_ctrl.get_integrator();
			rate_ctrl_status.pitchspeed_integ = _pitch_ctrl.get_integrator();

			if (wheel_control) {
				rate_ctrl_status.additional_integ1 = _wheel_ctrl.get_integrator();

			} else {
				rate_ctrl_status.yawspeed_integ = _yaw_ctrl.get_integrator();
			}

			_rate_ctrl_status_pub.publish(rate_ctrl_status);
		}

		// Add feed-forward from roll control output to yaw control output
		// This can be used to counteract the adverse yaw effect when rolling the plane
		_actuators.control[actuator_controls_s::INDEX_YAW] += _param_fw_rll_to_yaw_ff.get()
				* constrain(_actuators.control[actuator_controls_s::INDEX_ROLL], -1.0f, 1.0f);

		_actuators.control[actuator_controls_s::INDEX_FLAPS] = _flaps_applied;
		_actuators.control[5] = _manual_control_setpoint.aux1;
		_actuators.control[actuator_controls_s::INDEX_AIRBRAKES] = _flaperons_applied;
		// FIXME: this should use _vcontrol_mode.landing_gear_pos in the future
		_actuators.control[7] = _manual_control_setpoint.aux3;

		/* lazily publish the setpoint only once available */
		_actuators.timestamp = hrt_absolute_time();
		_actuators.timestamp_sample = att.timestamp;

		/* Only publish if any of the proper modes are enabled */
		if (_vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled ||
		    _vcontrol_mode.flag_control_manual_enabled) {
			_actuators_0_pub.publish(_actuators);
		}
	}

	perf_end(_loop_perf);
}

void FixedwingAttitudeControl::control_flaps(const float dt)
{
	/* default flaps to center */
	float flap_control = 0.0f;

	/* map flaps by default to manual if valid */
	if (PX4_ISFINITE(_manual_control_setpoint.flaps) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_param_fw_flaps_scl.get()) > 0.01f) {
		flap_control = 0.5f * (_manual_control_setpoint.flaps + 1.0f) * _param_fw_flaps_scl.get();

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_param_fw_flaps_scl.get()) > 0.01f) {

		switch (_att_sp.apply_flaps) {
		case vehicle_attitude_setpoint_s::FLAPS_OFF:
			flap_control = 0.0f; // no flaps
			break;

		case vehicle_attitude_setpoint_s::FLAPS_LAND:
			flap_control = 1.0f * _param_fw_flaps_scl.get() * _param_fw_flaps_lnd_scl.get();
			break;

		case vehicle_attitude_setpoint_s::FLAPS_TAKEOFF:
			flap_control = 1.0f * _param_fw_flaps_scl.get() * _param_fw_flaps_to_scl.get();
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
	if (PX4_ISFINITE(_manual_control_setpoint.aux2) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_param_fw_flaperon_scl.get()) > 0.01f) {

		flaperon_control = 0.5f * (_manual_control_setpoint.aux2 + 1.0f) * _param_fw_flaperon_scl.get();

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_param_fw_flaperon_scl.get()) > 0.01f) {

		if (_att_sp.apply_flaps == vehicle_attitude_setpoint_s::FLAPS_LAND) {
			flaperon_control = _param_fw_flaperon_scl.get();

		} else {
			flaperon_control = 0.0f;
		}
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
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingAttitudeControl *instance = new FixedwingAttitudeControl(vtol);

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

	PRINT_MODULE_USAGE_NAME("fw_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[])
{
	return FixedwingAttitudeControl::main(argc, argv);
}

void FixedwingAttitudeControl::JUAN_position_control()
{
	// Not variables; Make these constants later
	float _gravity_const = 9.81f;
	float _mass_const = 0.45f;

	/* --- Real life --- */
	// float KpX = 0.75f*0.6f*1.0f*3*0.1f*1.8f;
	// float KpY = 0.75f*0.6f*1.0f*3*0.1f*1.8f;
	// float KpZ = 0.75f*0.7f*6*0.1f*6.0f;
	// float KdX = 0.7f*0.9f*1.0f*0.5f*0.42f;
	// float KdY = 0.7f*0.9f*1.0f*0.5f*0.42f;
	// float KdZ = 0.7f*0.9f*1.0f*0.5f*0.21f;
	// float KiX = 0.25f*0.0008f;
	// float KiY = 0.25f*0.0008f;
	// float KiZ = 0.25f*0.0004f;

	/* --- SITL --- */
	float KpX = 0.54f / ( (1.0f*0.8f*0.8f) ) ;
	float KpY = 0.54f / ( (1.0f*0.8f*0.8f) ) ;
	float KpZ = 0.54f / ( (1.0f*0.8f*0.8f) * 1.2f) ;
	float KdX = 0.336f / (1.0f*0.8f*0.8f*(1.3f));
	float KdY = 0.336f / (1.0f*0.8f*0.8f*(1.3f));
	float KdZ = 0.168f / (1.0f*0.8f*0.8f*(1.2f));
	float KiX = 0.25f*0.0008f;
	float KiY = 0.25f*0.0008f;
	float KiZ = 0.25f*0.0004f;

	/* --- ⛽ integral gains --- */
	KiX *= 0.0f;
	KiY *= 0.0f;
	KiZ *= 0.0f;

	// Added roll gains
	// float k_roll_p = 35*0.005f*4.32f;
	// float k_roll_y = 0.5f*0.7f*0.02f*0.4f;
	// float max_roll = 30.0f;
	// float k_roll_i = 0.0f*0.01f;

	// Added roll gains SITL!
	float k_roll_p = 4.32f;
	float k_roll_y = 0.2f;
	// float max_roll = 30.0f;



	// Assigned measured position and velocity
	_local_pos_sub.update(&_local_pos);
	float _pos_x_est = _local_pos.x;
	float _pos_y_est = _local_pos.y;
	float _pos_z_est = _local_pos.z;
	float _vel_x_est = _local_pos.vx;
	float _vel_y_est = _local_pos.vy;
	float _vel_z_est = _local_pos.vz;

	// Call JUAN Maneuver generator. This assigns a position setpoint.

	// NOTE!!!!!!!!!!! Check Qground disarm parameters!!!!!!
	 JUAN_reference_generator(6); //3 == zigzag

	// Control law
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
	float Fv1 = 1.0f*0.8f*0.8f*(_acc_x_ref + 1.3f*KdX * _error_vel_x + KpX * _error_pos_x + 0.5f*KiX * _error_x_int);
	float Fv2 = 1.0f*0.8f*0.8f*(_acc_y_ref + 1.3f*KdY * _error_vel_y + KpY * _error_pos_y + 0.5f*KiY * _error_y_int);
	float Fv3 = 1.0f*0.8f*0.8f*(1.2f*KdZ * _error_vel_z + 1.2f*KpZ * _error_pos_z + 0.3f*KiZ * _error_z_int) - 0.4f*_gravity_const;
	float _norm_F = sqrtf(Fv1*Fv1+Fv2*Fv2+Fv3*Fv3);
	float fv1 = Fv1/_norm_F;
	float fv2 = Fv2/_norm_F;
	float fv3 = Fv3/_norm_F;
	// Thrust magnitude (N)
	ThrustN = _mass_const*_norm_F;
	// Non-normalized wing vector
	float Wv1 = -fv2;
	float Wv2 = fv1;

	float _vel_xy_ref = sqrtf(_vel_x_ref*_vel_x_ref+_vel_y_ref*_vel_y_ref);
	float _norm_W = sqrtf(Wv1*Wv1+Wv2*Wv2);
	float angle_test = asinf(_norm_W);
	JUAN_singularity_management(_vel_xy_ref,angle_test);
	// _control_operation_mode = 0; //remove

	if (_control_operation_mode < 1)
	{
		// _juan_att_var.flight_mode_granular = 0;
			float wv1 = Wv1/_norm_W;
			float wv2 = Wv2/_norm_W;
			float wv3 = 0.0f;

			float proj1 = wv3*fv2-wv2*fv3;
			float proj2 = wv1*fv3-wv3*fv1;
			float proj3 = wv2*fv1-wv1*fv2;

			belly_n_old = proj1;
			belly_e_old = proj2;

				float psi = atan2f(_local_pos.y, _local_pos.x);
				float Ye = sinf(psi)*(_pos_x_ref - _local_pos.x) + cosf(psi)*(_pos_y_ref - _local_pos.y);
				float psi_c = k_roll_y * Ye;

				PX4_INFO("psi: %f , psi_c: %f, Ye: %f", (double)psi, (double)psi_c, (double)Ye);

				/* ----- UNWRAP ----- */
				if (psi_c < -PI_f)
				{
					psi_c = psi_c + 2.0f*PI_f;
				}
				else if (psi_c > PI_f)
				{
					psi_c = psi_c - 2.0f*PI_f;
				}

				float max_roll = 20.0f; //from matlab
				float roll_com = (0.5f*k_roll_p*psi_c);
				if (roll_com >= 0.0f)
				{
					if (roll_com > max_roll*PI_f/180.0f)
					{
						roll_com = max_roll*PI_f/180.0f;
					}
				}
				else {
					if (roll_com < -max_roll*PI_f/180)
					{
						roll_com = -max_roll*PI_f/180;
					}
				}
				_juan_att_var.roll_comm = roll_com;

				float m_ba11 = fv1;
				float m_ba12 = fv2;
				float m_ba13 = fv3;

				float m_ba21 =  wv1*cosf(roll_com) + proj1*sinf(roll_com);
				float m_ba22 =  wv2*cosf(roll_com) + proj2*sinf(roll_com);
				float m_ba23 =  wv3*cosf(roll_com) + proj3*sinf(roll_com);

				float m_ba31 =  -wv1*sinf(roll_com) + proj1*cosf(roll_com);
				float m_ba32 =  -wv2*sinf(roll_com) + proj2*cosf(roll_com);
				float m_ba33 =  -wv3*sinf(roll_com) + proj3*cosf(roll_com);

				float bldr_array_cri[9] = {m_ba11,m_ba21,m_ba31,m_ba12,m_ba22,m_ba32,m_ba13,m_ba23,m_ba33};
				matrix::SquareMatrix<float, 3> Bldr_Matrix_cri(bldr_array_cri);
				matrix::Dcmf C_ref_steady (Bldr_Matrix_cri);
				C_ri_pos = C_ref_steady;
	}
	else
	{
				_juan_att_var.flight_mode_granular = 1;
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
		_juan_att_var.flight_mode_granular = 2;
		if (xy_speed > vel_thrs_sup)
		{
			if (angle_vect > angle_thrs_sup)
			{
				_juan_att_var.flight_mode_granular = 3;
				_control_operation_mode = 0;
			}
			else{
				_juan_att_var.flight_mode_granular = 4;
				_control_operation_mode = 1;
			}
		}
		else{
			_juan_att_var.flight_mode_granular = 5;
			_control_operation_mode = 1;
		}
	}
	else //in normal mode
	{
		_juan_att_var.flight_mode_granular = 6;
		if (xy_speed > vel_thrs_inf)
		{
			if (angle_vect > angle_thrs_inf)
			{
				_juan_att_var.flight_mode_granular = 7;
				_control_operation_mode = 0;
			}
			else{
				_juan_att_var.flight_mode_granular = 8;
				_control_operation_mode = 1;
			}
		}
		else{
				_juan_att_var.flight_mode_granular = 9;
				_control_operation_mode = 1;
		}
	}
}

void FixedwingAttitudeControl::JUAN_reference_generator(int _maneuver_type)
{
	if (_maneuver_type == 1)
	{
	float t_man = _time_elapsed;
	float Vel_track1 = 10.0f;
	    _vel_x_ref = Vel_track1*cosf(_initial_heading);
	    _vel_y_ref = Vel_track1*sinf(_initial_heading);
	    _vel_z_ref = 0.0f;

	    _pos_x_ref = _pos_x_initial+Vel_track1*cosf(_initial_heading)*t_man;
	    _pos_y_ref = _pos_y_initial+Vel_track1*sinf(_initial_heading)*t_man;
	    _pos_z_ref = _pos_z_initial;
	// }
	}
	else if (_maneuver_type == 2)
	{
		float t_man = _time_elapsed;
		float V_i = 5.0f;
		float t_init = 2.0;
		float t_stop = 2.0f;
		float a_slow = -V_i/t_stop;
		if (t_man < t_init)
		{
			_vel_x_ref = V_i*cosf(_initial_heading); //straight line
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
		}
		else {
			_vel_x_ref = 0.0f;
			_vel_y_ref = 0.0f;
			_vel_z_ref = 0.0f;

			float pos_mag = a_slow/2*t_stop*t_stop+V_i*t_stop+V_i*t_init;
			_pos_x_ref = pos_mag*cosf(_initial_heading)+_pos_x_initial;
			_pos_y_ref = pos_mag*sinf(_initial_heading)+_pos_y_initial;
			_pos_z_ref = _pos_z_initial;

		}

	}
	else if (_maneuver_type == 3) //Jackson's path, zigzag
	{
		float t_man = _time_elapsed;
		float t_turns = 7.5f; //time allowed for each sucessive straight run

		float V_i = 10.0f;

		if (turnCount < 8 && !completeFlag)
		{
			// PX4_INFO("t_man : %f", t_man);
			if (t_man - t_last_vertex > t_turns) //if it's time to turn
			{
				_initial_heading += PI_f/2.0f; //Rotate 90
				_pos_x_last_vtx = _pos_x_ref - _pos_x_initial;
				_pos_y_last_vtx = _pos_y_ref - _pos_y_initial;
				t_last_vertex = t_man;
				turnCount++;

				PX4_INFO("turn %i", turnCount);

				if(turnCount == 4){feedforward_flag = false;} //turn ff on
			}
			_vel_x_ref = V_i*cosf(_initial_heading);
			_vel_y_ref = V_i*sinf(_initial_heading);
			_vel_z_ref = 0.0f;
			// PX4_INFO("t_man : %f", t_man);
			_pos_x_ref = _pos_x_initial + V_i*cosf(_initial_heading)*(t_man - t_last_vertex) + _pos_x_last_vtx;
			_pos_y_ref = _pos_y_initial + V_i*sinf(_initial_heading)*(t_man - t_last_vertex) + _pos_y_last_vtx;
			_pos_z_ref = _pos_z_initial;

			t_last = t_man;


		}
		else { //Go into hover
			completeFlag = true;
			if(!exitMsgSent)
			{
				PX4_INFO("Path exiting");
				exitMsgSent = true;
			}


			/* ---- just keep swimming ---- */
			_vel_x_ref = V_i*cosf(_initial_heading);
			_vel_y_ref = V_i*sinf(_initial_heading);
			_vel_z_ref = 0.0f;

			_pos_x_ref = _pos_x_initial + V_i*cosf(_initial_heading)*(t_man - t_last); //need some t_end for this
			_pos_y_ref = _pos_y_initial + V_i*sinf(_initial_heading)*(t_man - t_last);
			_pos_z_ref = _pos_z_initial;

			/* -- for resetting path -- */
			_pos_x_last_vtx = 0.0f;
			_pos_y_last_vtx = 0.0f;

			t_last_vertex = 0.0f;
			turnCount = 0;


			/* ---- reset feedforward stuff ---- */
			feedforward_flag = true;
			// _juan_att_var.feedforward_on = true;

		}
	}
	else if(_maneuver_type == 4) //Straight path, turn wff on and off
	{
		float t_man = _time_elapsed;
		// float Vel_track1 = 10.0f;
		float Vel_track1 = _initial_vxy;
		float t_switch_ff = 40.0f;

		if(t_man < t_switch_ff){ feedforward_flag = true; }
		else { feedforward_flag = false; if(!exitMsgSent){{PX4_INFO("Switching to no-feedforward"); exitMsgSent = true;}}}

		_vel_x_ref = Vel_track1*cosf(_initial_heading);
		_vel_y_ref = Vel_track1*sinf(_initial_heading);
		_vel_z_ref = 0.0f;

		_pos_x_ref = _pos_x_initial+Vel_track1*cosf(_initial_heading)*t_man;
		_pos_y_ref = _pos_y_initial+Vel_track1*sinf(_initial_heading)*t_man;
		_pos_z_ref = _pos_z_initial;

		float eVecN = _pos_x_ref - _local_pos.x;
		float eVecE = _pos_y_ref - _local_pos.y;

		_juan_att_var.path_rel_err = eVecN*cosf(_initial_heading + PI_f/2.0f) +  eVecE*sinf(_initial_heading + PI_f/2.0f);

	}
	else if(_maneuver_type == 5) //Jackson's path, rectangle
	{
		// feedforward_flag = false;
		float t_turn = 7.5; //Useless
		float t_man = _time_elapsed;
		float t_turns_long  = 7.5f;
		float t_turn_short = 3.0f;
		float V_i = 10.0f;
		// V_i = _initial_vxy;

		if (turnCount < 16 && !completeFlag)
		{
			//Flip between long and short runs
			if(longTurn){
				t_turn = t_turns_long;
			}else{
				t_turn = t_turn_short;
			}


			if (t_man - t_last_vertex > t_turn) //if it's time to turn
			{
				_initial_heading += PI_f/2.0f; //Rotate 90
				_pos_x_last_vtx = _pos_x_ref - _pos_x_initial;
				_pos_y_last_vtx = _pos_y_ref - _pos_y_initial;
				t_last_vertex = t_man;
				turnCount++;

				PX4_INFO("turn %i", turnCount);

				if(turnCount == 8){feedforward_flag = false;} //turn ff on
				longTurn = !longTurn;
			}
			_vel_x_ref = V_i*cosf(_initial_heading);
			_vel_y_ref = V_i*sinf(_initial_heading);
			_vel_z_ref = 0.0f;
			// PX4_INFO("t_man : %f", t_man);
			_pos_x_ref = _pos_x_initial + V_i*cosf(_initial_heading)*(t_man - t_last_vertex) + _pos_x_last_vtx;
			_pos_y_ref = _pos_y_initial + V_i*sinf(_initial_heading)*(t_man - t_last_vertex) + _pos_y_last_vtx;
			_pos_z_ref = _pos_z_initial;

			t_last = t_man;
			float eVecN = _pos_x_ref - _local_pos.x;
			float eVecE = _pos_y_ref - _local_pos.y;

			_juan_att_var.path_rel_err = eVecN*cosf(_initial_heading + PI_f/2.0f) +  eVecE*sinf(_initial_heading + PI_f/2.0f);
		}
		else { //Go into hover
			completeFlag = true;
			if(!exitMsgSent)
			{
				PX4_INFO("Path exiting");
				exitMsgSent = true;
			}


			/* ---- just keep swimming ---- */
			_vel_x_ref = V_i*cosf(_initial_heading);
			_vel_y_ref = V_i*sinf(_initial_heading);
			_vel_z_ref = 0.0f;

			_pos_x_ref = _pos_x_initial + V_i*cosf(_initial_heading)*(t_man - t_last); //need some t_end for this
			_pos_y_ref = _pos_y_initial + V_i*sinf(_initial_heading)*(t_man - t_last);
			_pos_z_ref = _pos_z_initial;

			/* -- for resetting path -- */
			_pos_x_last_vtx = 0.0f;
			_pos_y_last_vtx = 0.0f;

			t_last_vertex = 0.0f;
			turnCount = 0;


			/* ---- reset feedforward stuff ---- */
			feedforward_flag = true;
			// _juan_att_var.feedforward_on = true;

		}
	}
	else if(_maneuver_type == 6) //Jackson's path, circle
	{
		float V_n = _initial_vxy;
		V_n = 10.0f;
		float radius = 30.0f; //m
		float t_runup = 10.0f; //sec
		float discrep = 0.0f; //m

		// Center of circle
		float _x_zero =  _pos_x_exit + (radius - discrep) * cosf(_initial_heading - PI_f/2);
		float _y_zero = _pos_y_exit + (radius - discrep) * sinf(_initial_heading - PI_f/2);

		float delX = _x_zero - _pos_x_exit;
		float delY = _y_zero - _pos_y_exit;

		float maxRot = 6; // maximum number of revolutions
		float t_man = _time_elapsed;

		float theta_0 = atan2f(delY, delX) + PI_f;

		if(!PX4_ISFINITE(theta_0)){theta_0 = 0.0f; PX4_INFO("Non finite theta_0, using 0 instead");}

		// float V_i = 10.0f;


		float pos_x_final = 0;
		float pos_y_final = 0;

		if(t_man < t_runup)
		{
			feedforward_flag = true;
			_vel_x_ref = V_n*cosf(_initial_heading);
			_vel_y_ref = V_n*sinf(_initial_heading);
			_vel_z_ref = 0.0f;

			_pos_x_ref = _pos_x_initial + V_n*cosf(_initial_heading)*t_man;
			_pos_y_ref = _pos_y_initial + V_n*sinf(_initial_heading)*t_man;
			_pos_z_ref = _pos_z_initial;

			_pos_x_exit = _pos_x_ref;
			_pos_y_exit = _pos_y_ref;
			t_circ = t_man;
			float eVecN = _pos_x_ref - _local_pos.x;
			float eVecE = _pos_y_ref - _local_pos.y;
			_juan_att_var.path_rel_err = eVecN*cosf(_initial_heading + PI_f/2.0f) +  eVecE*sinf(_initial_heading + PI_f/2.0f);
			_juan_att_var.estimated_position_ff_x = _local_pos.x;
			_juan_att_var.estimated_position_ff_y = _local_pos.y;

		}
		else
		{
			float theta_i = -V_n/radius * (t_man - t_circ) + theta_0;
			if (abs(theta_i) < 2.0f * PI_f * maxRot)
			{
				_acc_x_ref = 0.0f * -((V_n * V_n)/radius) * cosf(theta_i);
				_acc_y_ref = 0.0f * -((V_n * V_n)/radius) * sinf(theta_i);

				_juan_att_var.reference_acceleration_x = _acc_x_ref;
				_juan_att_var.reference_acceleration_y = _acc_y_ref;

				_vel_x_ref = V_n * sinf(theta_i);
				_vel_y_ref = -V_n * cosf(theta_i);
				_vel_z_ref = 0.0f;

				_pos_x_ref =_x_zero + radius*cosf(theta_i);
				_pos_y_ref =_y_zero + radius*sinf(theta_i);
				_pos_z_ref = _pos_z_initial;

				t_last = t_man;

				if(abs(theta_i) > PI_f * maxRot){
					feedforward_flag = false;
					_juan_att_var.estimated_position_nff_x = _local_pos.x;
					_juan_att_var.estimated_position_nff_y = _local_pos.y;
					if(!exitMsgSent){PX4_INFO("FF off"); exitMsgSent=true;}
				}else{
					feedforward_flag = true;
					_juan_att_var.estimated_position_ff_x = _local_pos.x;
					_juan_att_var.estimated_position_ff_y = _local_pos.y;
				}


				/* --- ensuring a clean exit --- */
				t_last = t_man;
				pos_x_final = _pos_x_ref;
				pos_y_final = _pos_y_ref;
			}
			else { //Go into hover
				completeFlag = true;
				if(!exitMsgSent)
				{
					PX4_INFO("Path exiting");
					exitMsgSent = true;
				}

				/* ---- just keep swimming ---- */
				_vel_x_ref = V_n*sinf(theta_i);
				_vel_y_ref = -V_n*cosf(theta_i);
				_vel_z_ref = 0.0f;

				_pos_x_ref = pos_x_final + V_n*cosf(theta_i)*(t_man - t_last); //need some t_end for this
				_pos_y_ref = pos_y_final + V_n*sinf(theta_i)*(t_man - t_last);
				_pos_z_ref = _pos_z_initial;

				/* ---- reset feedforward stuff ---- */
				feedforward_flag = true;
				// _juan_att_var.feedforward_on = true;
			}
		}


	}


}

float FixedwingAttitudeControl::saturate(float value, float min, float max)
{
  float output = value;
  if (value < min){output = min;}
  if (value > max){output = max;}
  return output;
}

void FixedwingAttitudeControl::wind_ff_rot_update()
{

	/* ---- Wind vector ---- */
	float v_wind_N = _wind.windspeed_north;
	float v_wind_E = _wind.windspeed_east;
	// _local_pos_sub.update(&_local_pos);
	// float v_wind_N = -5.0f;
	// float v_wind_E = 0.0f;

	/* ---- Velocity vector ---- */

	float v_N = _local_pos.vx;
	float v_E = _local_pos.vy;

	float v_norm = sqrtf(v_N*v_N + v_E*v_E);

	float v_N_u = v_N/v_norm;
	float v_E_u = v_E/v_norm;

	float v_crs_z_N = -v_E_u;
	float v_crs_z_E = v_N_u;

	/* ---- rotation inertial to velocity vector ---- */

	float bldr_array_Cvii[9] = {v_N_u, v_E_u, 0.0f, v_crs_z_N, v_crs_z_E, 0.0f, 0.0f, 0.0f, 1.0f};
	matrix::SquareMatrix<float, 3> Bldr_Matrix_Cvii(bldr_array_Cvii);
	matrix::Dcmf Cvii (Bldr_Matrix_Cvii); // DCM cast transposes?
	Cvii = Cvii.transpose();

	/* ---- rotation inertial to relative velocity vector ---- */

	float v_tild_N = v_N - v_wind_N; //North rel vel
	float v_tild_E = v_E - v_wind_E; //East rel vel

	float v_tild_norm = sqrtf(v_tild_N*v_tild_N + v_tild_E*v_tild_E);

	float v_tild_N_u = v_tild_N / v_tild_norm;
	float v_tild_E_u = v_tild_E / v_tild_norm;

	float v_tild_crs_z_N = -v_tild_E_u;
	float v_tild_crs_z_E = v_tild_N_u;

	float bldr_array_Cv_tildi[9] = {v_tild_N_u, v_tild_E_u, 0.0f, v_tild_crs_z_N, v_tild_crs_z_E, 0.0f, 0.0f, 0.0f, 1.0f};
	matrix::SquareMatrix<float, 3> Bldr_Matrix_Cv_tildi(bldr_array_Cv_tildi);
	matrix::Dcmf Cv_tildi (Bldr_Matrix_Cv_tildi); // DCM cast transposes?
	Cv_tildi = Cv_tildi.transpose();

	matrix::Dcmf Civ_tild = Cv_tildi.transpose();

	R_wind = Cvii * Civ_tild;

	_juan_att_var.crab_angle_ff = asinf(  R_wind(1,0) );

	if(true)//(v_wind_E != 0.0f || v_wind_N != 0.0f) && (v_E != 0.0f || v_N != 0.0f))
	{
		_juan_att_var.crab_angle_ff_alt = acosf( (v_N * v_tild_N + v_E * v_tild_E) / ( v_tild_norm * v_norm ) );
	}

	_juan_att_var.r_wind_rows[0] =  R_wind(0,0);
	_juan_att_var.r_wind_rows[1] =  R_wind(0,1);
	_juan_att_var.r_wind_rows[2] =  R_wind(0,2);
	_juan_att_var.r_wind_rows[3] =  R_wind(1,0);
	_juan_att_var.r_wind_rows[4] =  R_wind(1,1);
	_juan_att_var.r_wind_rows[5] =  R_wind(1,2);
	_juan_att_var.r_wind_rows[6] =  R_wind(2,0);
	_juan_att_var.r_wind_rows[7] =  R_wind(2,1);
	_juan_att_var.r_wind_rows[8] =  R_wind(2,2);

	_juan_att_var.v_tild_n = v_tild_N;
	_juan_att_var.v_tild_e = v_tild_E;

	_juan_att_var.v_n = v_N;
	_juan_att_var.v_e = v_E;
}
