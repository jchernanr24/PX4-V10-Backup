uint64 timestamp		# time since system start (microseconds)


float32 attitude_error_function      # attitude error function used

float32[3] error_so3        # SO(3) error
float32[3] tau_ref          # Control torque
float32[3] ctr_defl          # deflections (degrees?)
float32[3] pwm_out_ctr          # PWM command
float32[9] cbi_rows          # Cbi rows first
float32[9] cri_rows          # Cri rows first

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

float32 estimated_velocity_x # north vel estimated
float32 estimated_velocity_y # east vel estimated
float32 estimated_velocity_z # down vel estimated



float32 control_status # singularity management

float32 true_airspeed # true airspeed tests
float32 indicated_airspeed #indicated airspeed test
float32 j_advance #advance ratio
float32 omega_rpm #previous omega

float32 crab_angle_ff #Angle of FF crab rotation matrix (R_wind)
bool feedforward_on #true if ff is on

float32[9] r_wind_rows #Rows of R_wind_mtx
float32[9] c_ri_pre_ff

float32 v_tild_n #Relative wind north N
float32 v_tild_e #Relative wind east E

float32 v_n #Ground speed north N
float32 v_e #Ground speed east E
