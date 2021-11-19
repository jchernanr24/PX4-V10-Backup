uint64 timestamp		# time since system start (microseconds)


float32 attitude_error_function      # attitude error function used

float32[3] error_so3        # SO(3) error
float32[3] tau_ref          # Control torque
float32[3] ctr_defl          # deflections (degrees?)
float32[3] pwm_out_ctr          # PWM command
float32[9] cbi_rows          # Cbi rows first
float32[9] cri_rows          # Cri rows first
float32[3] virtual_force     # f

float32 motor_rpm           #motor commanded rpm

float32 coordinate_yaw_rate # coordinated turn rate
float32 pilot_roll_com      # commanded roll manual
float32 pilot_pitch_com      # commanded pitch manual
float32 pilot_yaw_com      # commanded yaw manual
float32 ground_speed_attitude # ground speed for coordinated turn

float32 yaw_measured     # yaw angle for comparison
float32 pitch_measured   # pitch angle for comparison
float32 roll_measured   # roll angle for comparison

float32 yaw_reference     # yaw angle reference
float32 pitch_reference   # pitch angle reference
float32 roll_reference   # roll angle reference

float32[3] omega_ref   # angular velocity reference in ref coord
float32[3] omega_bod   # angular velocity measurement in body coord


float32 test_variable    # dummy tester slot


float32 reference_position_x # north reference
float32 reference_position_y # east reference
float32 reference_position_z # down reference

float32 estimated_position_x # north estimated
float32 estimated_position_y # east estimated
float32 estimated_position_z # down estimated

float32 reference_velocity_x # north vel reference
float32 reference_velocity_y # east vel reference
float32 reference_velocity_z # down vel reference

float32 estimated_velocity_x # north vel estimated
float32 estimated_velocity_y # east vel estimated
float32 estimated_velocity_z # down vel estimated

float32 normalized_throttle # throttle output

float32 control_status # singularity management

float32 true_airspeed # true airspeed tests
float32 indicated_airspeed #indicated airspeed test
float32 j_advance #advance ratio
float32 omega_rpm #previous omega
float32 heading_0 #initial heading upon switch to acro

float32 path_law # path law

float32 tangent_v1 # pf path tangent1
float32 tangent_v2 # pf path tangent2
float32 tangent_v3 # pf path tangent3

float32 lateral_err # lateral error used for added roll

float32 sigma_path # path parameter
float32 sigma_dot # path evol law

float32 error_s # ontrack error
float32 error_c # crosstrack error
float32 error_h # altitude error

float32 read_pitch #pitch stick value
float32 read_roll #roll stick value
float32 read_yaw #yaw stick value
float32 read_thrust #thrust stick value

float32 slipstr_att #slipstream estimate
