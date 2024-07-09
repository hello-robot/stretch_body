#Robot parameters for Stretch 3

# ######################### USER PARAMS ##################################################
#Template for the generated file: stretch_user_params.yaml
user_params_header='#User parameters\n' \
                   '#You can override nominal settings here\n' \
                   '#USE WITH CAUTION. IT IS POSSIBLE TO CAUSE UNSAFE BEHAVIOR OF THE ROBOT \n'

user_params_template={
    'robot': {'use_collision_manager': 0}} #Include this just as an example

# ###################### CONFIGURATION PARAMS #####################################################
#Template for the generated file: stretch_configuration_params.yaml
#Configuration parameters may have variation across the fleet of RE2 robots
configuration_params_header='#Parameters that are specific to this robot\n' \
                            '#Do not edit, instead edit stretch_user_params.yaml\n'

configuration_params_template={
    'arm':{
        'contact_models':{
            'effort_pct':{
                'contact_thresh_default':[-55.0, 55.0],
                'contact_thresh_homing':[-55.0, 55.0]}},
        'range_m':[0.0,0.52]},
    'lift': {
        'contact_models':{
            'effort_pct': {
                'contact_thresh_default': [-15.0, 70.0],
                'contact_thresh_homing': [-15.0, 80.0]}},
        'i_feedforward': 1.2,
        'range_m': [0.0, 1.10]},
    'base':{
        'wheel_separation_m': 0.3153},
    'head_pan':{
        'range_t': [0, 3827],
        'zero_t': 1250},
    'head_tilt':{
        'range_t': [1775,3150],
        'zero_t': 2048},
    'hello-motor-arm':{'serial_no': 'NA'},
    'hello-motor-lift':{'serial_no': 'NA','gains':{'i_safety_feedforward':1.2}},
    'hello-motor-left-wheel':{'serial_no': 'NA'},
    'hello-motor-right-wheel':{'serial_no': 'NA'},
    'pimu':{
        'config':{
            'cliff_zero': [0.0, 0.0, 0.0, 0.0],
            'gyro_zero_offsets': [0.0, 0.0, 0.0],
            'gravity_vector_scale': 1.0,
            'mag_offsets': [0.0, 0.0, 0.0],
            'mag_softiron_matrix': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            'rate_gyro_vector_scale': 1.0}},
    'robot':{
        'batch_name': 'NA',
        'serial_no': 'NA',
        'd435i':{'serial_no':'NA'},
        'd405': {'serial_no': 'NA'},
        'model_name':'SE3'},
    'stretch_gripper':{
        'range_t': [0, 9102],
        'zero_t': 3279},
    'wacc':{'config':{
        'accel_gravity_scale': 1.0}},
    'wrist_yaw':{
        'range_t': [0,9340],
        'zero_t': 7136}}

# ###################### NOMINAL PARAMS #####################################################
#Parameters that are common across the SE3 fleet

# ######## EOA Joints ######
# We use a modular design of dictionaries so that different parameter sets
# can be easily managed depending on the configuation of the end-of-arm
# Eg, which joints and tools and versions of hardware are present

SE3_stretch_gripper_SG3={
        'range_pad_t': [100.0, -100.0],
        'flip_encoder_polarity': 0,
        'gr': 1.0,
        'id': 14,
        'max_voltage_limit': 15,
        'min_grip_strength': -125,
        'min_voltage_limit': 11,
        'gripper_conversion':{
            'finger_length_m':0.171,
            'open_aperture_m':0.09,
            'closed_aperture_m':0.0,
            'open_robotis':70.0,
            'closed_robotis':0.0},
        'motion':{
            'trajectory_vel_ctrl':1,
            'trajectory_vel_ctrl_kP':1.5,
            'default':{
              'accel': 10.0,
              'vel': 6.0},
            'fast':{
              'accel': 10.0,
              'vel': 8.0},
            'max':{
              'accel': 20,
              'vel': 20},
            'slow':{
              'accel': 4.0,
              'vel': 3.0},
            'trajectory_max': {
                'vel_r': 50.0,
                'accel_r': 100.0},
            'vel_brakezone_factor': 1},
        'set_safe_velocity': 1,
        'pid': [640.0,0,0],
        'pwm_homing': [-400, 0],
        'pwm_limit': 885,
        'req_calibration': 1,
        'return_delay_time': 0,
        'stall_backoff': 2.0,
        'stall_max_effort': 20.0,
        'stall_max_time': 0.5,
        'stall_min_vel': 0.1,
        'temperature_limit': 72,
        'usb_name': '/dev/hello-dynamixel-wrist',
        'use_multiturn': 1,
        'use_pos_current_ctrl':0,
        'retry_on_comm_failure': 1,
        'baud': 115200,
        'enable_runstop': 1,
        'disable_torque_on_stop': 1,
        'range_t': [0, 9102],
        'zero_t': 3279}

SE3_wrist_yaw_DW3={
        'flip_encoder_polarity': 1,
        'gr': 2.4,
        'id': 13,
        'max_voltage_limit': 15,
        'min_voltage_limit': 11,
        'motion':{
            'trajectory_vel_ctrl':1,
            'trajectory_vel_ctrl_kP':1.5,
            'default':{
              'accel': 3.0,
              'vel': 2.0},
            'fast':{
              'accel': 5.0,
              'vel': 2.5},
            'max':{
              'accel': 10,
              'vel': 6.0},
            'slow':{
              'accel': 1.5,
              'vel': 0.75},
              'trajectory_max': {
                  'vel_r': 3.0,
                  'accel_r': 3.0},
            'vel_brakezone_factor': 0.2},
        'set_safe_velocity': 1,
        'pid': [640,0,0],
        'pwm_homing': [-300,300],
        'pwm_limit': 885,
        'req_calibration': 1,
        'return_delay_time': 0,
        'stall_backoff': 0.017,
        'stall_max_effort': 20.0,
        'stall_max_time': 1.0,
        'stall_min_vel': 0.1,
        'temperature_limit': 72,
        'usb_name': '/dev/hello-dynamixel-wrist',
        'use_multiturn': 1,
        'use_pos_current_ctrl':0,
        'retry_on_comm_failure': 1,
        'baud': 115200,
        'enable_runstop': 1,
        'disable_torque_on_stop': 1,
        'range_pad_t': [100.0, -100.0],
        'range_t': [0,9340],
        'zero_t': 7136}

SE3_wrist_pitch_DW3={
        'flip_encoder_polarity': 1,
        'enable_runstop': 1,
        'gr': 1.0,
        'id': 15,
        'max_voltage_limit': 15,
        'min_voltage_limit': 11,
        'motion': {
            'trajectory_vel_ctrl': 1,
            'trajectory_vel_ctrl_kP': 1.5,
            'default': {'accel': 6.0, 'vel': 2.0},
            'fast': {'accel': 8.0, 'vel': 2.0},
            'max': {'accel': 10.0, 'vel': 3.0},
            'slow': {'accel': 4.0, 'vel': 1.0},
            'trajectory_max': {'accel_r': 16.0, 'vel_r': 8.0},
            'vel_brakezone_factor': 1},
        'set_safe_velocity': 1,
        'pid': [400, 0, 200],
        'pwm_homing': [0, 0],
        'pwm_limit': 885,
        'range_t': [730, 2048],
        'req_calibration': 0,
        'return_delay_time': 0,
        'stall_backoff': 0.017,
        'stall_max_effort': 25.0,
        'stall_max_time': 5.0,
        'stall_min_vel': 0.1,
        'temperature_limit': 72,
        'usb_name': '/dev/hello-dynamixel-wrist',
        'use_multiturn': 0,
        'use_pos_current_ctrl':1,
        'zero_t': 1024,
        'baud': 115200,
        'retry_on_comm_failure': 1,
        'disable_torque_on_stop': 0,
        'float_on_stop': 1,
        'current_float_A': -0.13,
        'current_limit_A': 2.5
    }

SE3_wrist_roll_DW3={
        'flip_encoder_polarity': 0,
        'enable_runstop': 1,
        'gr': 1.0,
        'id': 16,
        'max_voltage_limit': 16,
        'min_voltage_limit': 9,
        'motion': {
            'trajectory_vel_ctrl': 1,
            'trajectory_vel_ctrl_kP': 1.5,
            'default': {'accel': 8.0, 'vel': 2.0},
            'fast': {'accel': 10.0, 'vel': 3.0},
            'max': {'accel': 12, 'vel': 4.5},
            'slow': {'accel': 4.0, 'vel': 1.0},
            'trajectory_max': {'accel_r': 16.0, 'vel_r': 8.0},
            'vel_brakezone_factor': 1},
        'set_safe_velocity': 1,
        'pid': [800, 0, 0],
        'pwm_homing': [0, 0],
        'pwm_limit': 885,
        'range_t': [150, 3950],
        'req_calibration': 0,
        'return_delay_time': 0,
        'stall_backoff': 0.017,
        'stall_max_effort': 25.0,
        'stall_max_time': 2.0,
        'stall_min_vel': 0.1,
        'temperature_limit': 80,
        'usb_name': '/dev/hello-dynamixel-wrist',
        'use_multiturn': 0,
        'use_pos_current_ctrl':0,
        'zero_t': 2048,
        'baud': 115200,
        'retry_on_comm_failure': 1,
        'disable_torque_on_stop': 0,
        'float_on_stop': 0,
        'current_float_A': 0.04,
        'current_limit_A': 1.0}

SE3_wrist_roll_DW3_tablet = SE3_wrist_roll_DW3
SE3_wrist_roll_DW3_tablet['float_on_stop'] = 0

# ######### EndOfArm Defn ##############
"""
Define the EndOfArm DynamixelXChain parameters
Point to which joint devices & parameters to load for the chain
"""

SE3_eoa_wrist_dw3_tool_sg3={
        'py_class_name': 'EOA_Wrist_DW3_Tool_SG3',
        'py_module_name': 'stretch_body.end_of_arm_tools',
        'use_group_sync_read': 1,
        'retry_on_comm_failure': 1,
        'baud': 115200,
        'dxl_latency_timer': 64,
        'stow': {
            'arm': 0.0,
            'lift': 0.3,
            'wrist_pitch': -0.62,
            'wrist_roll': 0.0,
            'wrist_yaw': 3.0,
            'stretch_gripper':0.0
        },
        'collision_mgmt': {
            'k_brake_distance': {'wrist_pitch': 0.5, 'wrist_yaw': 0.25, 'wrist_roll': 0.25, 'stretch_gripper': 0.25},
            'collision_pairs': {
                'link_gripper_fingertip_left_TO_link_lift': {'link_pts': 'link_gripper_fingertip_left', 'link_cube': 'link_lift','detect_as': 'edges', 'cube_scale': 1.3},
                'link_gripper_fingertip_right_TO_link_lift': {'link_pts': 'link_gripper_fingertip_right', 'link_cube': 'link_lift','detect_as': 'edges', 'cube_scale': 1.3},
                'link_gripper_fingertip_left_TO_link_arm_l1': {'link_pts': 'link_gripper_fingertip_left', 'link_cube': 'link_arm_l1','detect_as': 'edges', 'cube_scale': 1.6},
                'link_gripper_fingertip_right_TO_link_arm_l1': {'link_pts': 'link_gripper_fingertip_right', 'link_cube': 'link_arm_l1','detect_as': 'edges', 'cube_scale': 1.6},
                'link_gripper_s3_body_TO_base_link': {'link_pts': 'link_gripper_s3_body', 'link_cube': 'base_link','detect_as': 'edges'},
                'link_wrist_pitch_TO_base_link': {'link_pts': 'link_wrist_pitch', 'link_cube': 'base_link','detect_as': 'edges'},
                'link_wrist_yaw_bottom_TO_base_link': {'link_pts': 'link_wrist_yaw_bottom', 'link_cube': 'base_link','detect_as': 'edges'},
                'link_gripper_finger_left_TO_base_link': {'link_pts': 'link_gripper_finger_left','link_cube': 'base_link', 'detect_as': 'edges'},
                'link_gripper_finger_right_TO_base_link': {'link_pts': 'link_gripper_finger_right','link_cube': 'base_link', 'detect_as': 'edges'},
                'link_gripper_fingertip_left_TO_base_link': {'link_pts': 'link_gripper_fingertip_left','link_cube': 'base_link', 'detect_as': 'edges', 'cube_scale': 1.3},
                'link_gripper_fingertip_right_TO_base_link': {'link_pts': 'link_gripper_fingertip_right','link_cube': 'base_link', 'detect_as': 'edges', 'cube_scale': 1.3},
                'link_gripper_s3_body_TO_link_arm_l0': {'link_pts': 'link_gripper_s3_body', 'link_cube': 'link_arm_l0','detect_as': 'edges','cube_scale': 1.2},
                'link_gripper_s3_body_TO_link_arm_l1': {'link_pts': 'link_gripper_s3_body', 'link_cube': 'link_arm_l1','detect_as': 'edges'},
                'link_gripper_s3_body_TO_link_head_tilt': {'link_cube': 'link_head_tilt', 'link_pts': 'link_gripper_s3_body','detect_as': 'edges'},
                'link_gripper_finger_left_TO_link_head_tilt': {'link_cube': 'link_head_tilt', 'link_pts': 'link_gripper_finger_left','detect_as': 'edges'},
                'link_gripper_finger_right_TO_link_head_tilt': {'link_cube': 'link_head_tilt', 'link_pts': 'link_gripper_finger_right','detect_as': 'edges'},
                'link_gripper_finger_right_link_arm_l0': {'link_pts': 'link_gripper_finger_right', 'link_cube': 'link_arm_l0','detect_as': 'edges'},
                'link_gripper_finger_left_TO_link_arm_l1': {'link_pts': 'link_gripper_finger_left', 'link_cube': 'link_arm_l1','detect_as': 'edges'},
                'link_gripper_finger_right_TO_link_arm_l1': {'link_pts': 'link_gripper_finger_right', 'link_cube': 'link_arm_l1','detect_as': 'edges'},
                'link_arm_l0_TO_base_link':{'link_pts': 'link_arm_l0', 'link_cube': 'base_link','detect_as': 'edges'},
                'link_wrist_pitch_TO_base_link':{'link_pts': 'link_wrist_pitch', 'link_cube': 'base_link','detect_as': 'edges', 'cube_scale': 1.3}},

            'joints': {'arm': [{'motion_dir': 'neg', 'collision_pair': 'link_gripper_fingertip_right_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_gripper_s3_body_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_wrist_pitch_TO_base_link'}],
                       'lift': [{'motion_dir': 'neg', 'collision_pair': 'link_gripper_s3_body_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_wrist_pitch_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_wrist_yaw_bottom_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_gripper_finger_left_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_gripper_finger_right_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_gripper_fingertip_left_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_gripper_fingertip_right_TO_base_link'},
                                {'motion_dir': 'pos','collision_pair': 'link_gripper_s3_body_TO_link_head_tilt'},
                                {'motion_dir': 'pos','collision_pair': 'link_gripper_finger_left_TO_link_head_tilt'},
                                {'motion_dir': 'pos','collision_pair': 'link_gripper_finger_right_TO_link_head_tilt'}],
                       'wrist_pitch': [{'motion_dir': 'neg', 'collision_pair': 'link_gripper_s3_body_TO_base_link'},
                                       {'motion_dir': 'neg', 'collision_pair': 'link_gripper_finger_left_TO_base_link'},
                                       {'motion_dir': 'neg','collision_pair': 'link_gripper_finger_right_TO_base_link'},
                                       {'motion_dir': 'neg','collision_pair': 'link_gripper_fingertip_left_TO_base_link'},
                                       {'motion_dir': 'neg','collision_pair': 'link_gripper_fingertip_right_TO_base_link'},
                                       {'motion_dir': 'neg','collision_pair': 'link_gripper_s3_body_TO_link_head_tilt'},
                                       {'motion_dir': 'neg','collision_pair': 'link_gripper_finger_left_TO_link_head_tilt'},
                                       {'motion_dir': 'neg','collision_pair': 'link_gripper_finger_right_TO_link_head_tilt'},
                                       {'motion_dir': 'pos', 'collision_pair': 'link_gripper_s3_body_TO_link_arm_l0'},
                                       {'motion_dir': 'pos', 'collision_pair': 'link_gripper_s3_body_TO_link_arm_l1'},
                                       {'motion_dir': 'pos','collision_pair': 'link_gripper_s3_body_TO_link_head_tilt'},
                                       {'motion_dir': 'pos','collision_pair': 'link_gripper_finger_left_TO_link_head_tilt'},
                                       {'motion_dir': 'pos','collision_pair': 'link_gripper_finger_right_TO_link_head_tilt'},
                                       {'motion_dir': 'pos','collision_pair': 'link_gripper_fingertip_left_TO_link_lift'},
                                       {'motion_dir': 'pos','collision_pair': 'link_gripper_fingertip_right_TO_link_lift'}],
                       'wrist_roll': [{'motion_dir': 'neg', 'collision_pair': 'link_gripper_s3_body_TO_link_arm_l1'},
                                      {'motion_dir': 'neg', 'collision_pair': 'link_gripper_s3_body_TO_link_arm_l0'},
                                      {'motion_dir': 'pos', 'collision_pair': 'link_gripper_s3_body_TO_link_arm_l0'},
                                    #   {'motion_dir': 'pos', 'collision_pair': 'link_gripper_s3_body_TO_link_arm_l1'},
                                      {'motion_dir': 'pos', 'collision_pair': 'link_gripper_s3_body_TO_base_link'},
                                      {'motion_dir': 'neg', 'collision_pair': 'link_gripper_s3_body_TO_base_link'}],
                       'wrist_yaw': [{'motion_dir': 'pos', 'collision_pair': 'link_gripper_fingertip_left_TO_link_lift'},
                                     {'motion_dir': 'neg', 'collision_pair': 'link_gripper_finger_right_TO_base_link'},
                                     {'motion_dir': 'pos', 'collision_pair': 'link_gripper_finger_left_TO_base_link'},
                                    #  {'motion_dir': 'neg', 'collision_pair': 'link_gripper_s3_body_TO_link_arm_l0'},
                                     {'motion_dir': 'pos', 'collision_pair': 'link_gripper_s3_body_TO_link_arm_l1'},
                                    #  {'motion_dir': 'pos','collision_pair': 'link_gripper_s3_body_TO_link_head_tilt'},
                                     {'motion_dir': 'pos', 'collision_pair': 'link_gripper_s3_body_TO_base_link'},
                                     {'motion_dir': 'neg', 'collision_pair': 'link_gripper_s3_body_TO_base_link'},
                                     {'motion_dir': 'pos','collision_pair': 'link_gripper_finger_left_TO_link_head_tilt'},
                                     {'motion_dir': 'pos','collision_pair': 'link_gripper_finger_right_TO_link_head_tilt'},
                                     {'motion_dir': 'neg','collision_pair': 'link_gripper_s3_body_TO_link_head_tilt'},
                                     {'motion_dir': 'pos','collision_pair': 'link_gripper_finger_left_TO_link_head_tilt'},
                                     {'motion_dir': 'neg','collision_pair': 'link_gripper_finger_right_TO_link_head_tilt'},
                                     {'motion_dir': 'pos','collision_pair': 'link_gripper_finger_left_TO_link_arm_l1'},
                                     {'motion_dir': 'neg','collision_pair': 'link_gripper_finger_right_TO_link_arm_l1'}]}},

        'devices': {
            'wrist_pitch': {
                'py_class_name': 'WristPitch',
                'py_module_name': 'stretch_body.wrist_pitch',
                'device_params':'SE3_wrist_pitch_DW3'
            },
            'wrist_roll': {
                'py_class_name': 'WristRoll',
                'py_module_name': 'stretch_body.wrist_roll',
                'device_params':'SE3_wrist_roll_DW3'
            },
            'wrist_yaw': {
                'py_class_name': 'WristYaw',
                'py_module_name': 'stretch_body.wrist_yaw',
                'device_params':'SE3_wrist_yaw_DW3'
            },
            'stretch_gripper': {
                'py_class_name': 'StretchGripper3',
                'py_module_name': 'stretch_body.stretch_gripper',
                'device_params':'SE3_stretch_gripper_SG3'
            }
    }}


SE3_eoa_wrist_dw3_tool_nil={
        'py_class_name': 'EOA_Wrist_DW3_Tool_NIL',
        'py_module_name': 'stretch_body.end_of_arm_tools',
        'use_group_sync_read': 1,
        'retry_on_comm_failure': 1,
        'baud': 115200,
        'dxl_latency_timer': 64,
        'wrist': 'eoaw_dw3',
        'tool': 'eoat_nil',
        'stow': {
            'arm': 0.0,
            'lift': 0.3,
            'wrist_pitch': -0.62,
            'wrist_roll': 0.0,
            'wrist_yaw': 3.0
        },
        'collision_mgmt': {
            'k_brake_distance': {'wrist_pitch': 0.25, 'wrist_yaw': 0.25, 'wrist_roll': 0.25},
            'collision_pairs': {
                'link_wrist_pitch_TO_base_link': {'link_pts': 'link_wrist_pitch', 'link_cube': 'base_link','detect_as': 'pts'},
                'link_wrist_yaw_bottom_TO_base_link': {'link_pts': 'link_wrist_yaw_bottom', 'link_cube': 'base_link','detect_as': 'pts'}},
            'joints': {'lift': [{'motion_dir': 'neg', 'collision_pair': 'link_wrist_pitch_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_wrist_yaw_bottom_TO_base_link'}]}},

        'devices': {
            'wrist_pitch': {
                'py_class_name': 'WristPitch',
                'py_module_name': 'stretch_body.wrist_pitch',
                'device_params': 'SE3_wrist_pitch_DW3'
            },
            'wrist_roll': {
                'py_class_name': 'WristRoll',
                'py_module_name': 'stretch_body.wrist_roll',
                'device_params': 'SE3_wrist_roll_DW3'
            },
            'wrist_yaw': {
                'py_class_name': 'WristYaw',
                'py_module_name': 'stretch_body.wrist_yaw',
                'device_params': 'SE3_wrist_yaw_DW3'
            }
            }}


SE3_eoa_wrist_dw3_tool_tablet_12in={
        'py_class_name': 'EOA_Wrist_DW3_Tool_Tablet_12in',
        'py_module_name': 'stretch_body.end_of_arm_tools',
        'use_group_sync_read': 1,
        'retry_on_comm_failure': 1,
        'portrait_orientation': 0,
        'lock_wrist_roll': 0,
        'baud': 115200,
        'dxl_latency_timer': 64,
        'wrist': 'eoaw_dw3',
        'tool': 'eoat_nil',
        'stow': {
            'arm': 0.0,
            'lift': 0.3,
            'wrist_pitch': 0.0,
            'wrist_roll': 0.0,
            'wrist_yaw': 1.57
        },
        'collision_mgmt': {
            'k_brake_distance': {'wrist_pitch': 0.5, 'wrist_yaw':1.5, 'wrist_roll': 0.25},
            'collision_pairs': {
                'link_DW3_tablet_12in_TO_base_link': {'link_cube': 'base_link', 'link_pts': 'link_DW3_tablet_12in', 'detect_as': 'edges'},
                'link_DW3_tablet_12in_TO_link_arm_l0': {'link_cube': 'link_arm_l0', 'link_pts': 'link_DW3_tablet_12in','detect_as': 'edges'},
                'link_DW3_tablet_12in_TO_link_arm_l1': {'link_cube': 'link_arm_l1', 'link_pts': 'link_DW3_tablet_12in','detect_as': 'edges'},
                'link_DW3_tablet_12in_TO_link_head_tilt': {'link_cube': 'link_head_tilt', 'link_pts': 'link_DW3_tablet_12in','detect_as': 'edges', 'cube_scale': 1.8},
                'link_wrist_pitch_TO_base_link': {'link_pts': 'link_wrist_pitch', 'link_cube': 'base_link','detect_as': 'edges'},
                'link_wrist_yaw_bottom_TO_base_link': {'link_pts': 'link_wrist_yaw_bottom', 'link_cube': 'base_link','detect_as': 'edges'},
                'link_DW3_tablet_12in_TO_link_lift': {'link_pts': 'link_DW3_tablet_12in', 'link_cube': 'link_lift','detect_as': 'edges'},},

            'joints': {'arm':  [{'motion_dir': 'neg', 'collision_pair': 'link_DW3_tablet_12in_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_wrist_pitch_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_DW3_tablet_12in_TO_link_head_tilt'}],

                       'lift': [{'motion_dir': 'neg', 'collision_pair': 'link_DW3_tablet_12in_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_wrist_pitch_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_wrist_yaw_bottom_TO_base_link'},
                                {'motion_dir': 'pos', 'collision_pair': 'link_DW3_tablet_12in_TO_link_head_tilt'}],

                       'wrist_pitch': [{'motion_dir': 'neg', 'collision_pair': 'link_DW3_tablet_12in_TO_base_link'},
                                    #    {'motion_dir': 'pos', 'collision_pair': 'link_DW3_tablet_12in_TO_base_link'},
                                    #    {'motion_dir': 'pos', 'collision_pair': 'link_DW3_tablet_12in_TO_link_arm_l0'},
                                       {'motion_dir': 'pos', 'collision_pair': 'link_DW3_tablet_12in_TO_link_arm_l1'},
                                    #    {'motion_dir': 'neg', 'collision_pair': 'link_DW3_tablet_12in_TO_link_head_tilt'},
                                       {'motion_dir': 'pos', 'collision_pair': 'link_DW3_tablet_12in_TO_link_head_tilt'}
                                       ],

                       'wrist_roll': [{'motion_dir': 'neg', 'collision_pair': 'link_DW3_tablet_12in_TO_link_arm_l1'},
                                      {'motion_dir': 'pos', 'collision_pair': 'link_DW3_tablet_12in_TO_link_arm_l0'}],

                       'wrist_yaw': [
                                     {'motion_dir': 'neg', 'collision_pair': 'link_DW3_tablet_12in_TO_link_arm_l0'},
                                     {'motion_dir': 'neg', 'collision_pair': 'link_DW3_tablet_12in_TO_link_arm_l1'},
                                    #  {'motion_dir': 'neg', 'collision_pair': 'link_DW3_tablet_12in_TO_link_arm_l0'},
                                     {'motion_dir': 'pos', 'collision_pair': 'link_DW3_tablet_12in_TO_link_lift'},
                                     {'motion_dir': 'pos', 'collision_pair': 'link_DW3_tablet_12in_TO_link_arm_l1'},
                                     {'motion_dir': 'neg', 'collision_pair': 'link_DW3_tablet_12in_TO_base_link'},
                                     {'motion_dir': 'pos', 'collision_pair': 'link_DW3_tablet_12in_TO_base_link'},
                                     {'motion_dir': 'pos', 'collision_pair': 'link_DW3_tablet_12in_TO_link_head_tilt'},
                                    #  {'motion_dir': 'neg', 'collision_pair': 'link_DW3_tablet_12in_TO_link_head_tilt'},
                                     ],

                       'wrist_roll': [{'motion_dir': 'neg', 'collision_pair': 'link_DW3_tablet_12in_TO_link_arm_l0'},
                                     {'motion_dir': 'pos', 'collision_pair': 'link_DW3_tablet_12in_TO_link_arm_l1'}]
                        }
                        },

        'devices': {
            'wrist_pitch': {
                'py_class_name': 'WristPitch',
                'py_module_name': 'stretch_body.wrist_pitch',
                'device_params': 'SE3_wrist_pitch_DW3'
            },
            'wrist_roll': {
                'py_class_name': 'WristRoll',
                'py_module_name': 'stretch_body.wrist_roll',
                'device_params': 'SE3_wrist_roll_DW3_tablet'
            },
            'wrist_yaw': {
                'py_class_name': 'WristYaw',
                'py_module_name': 'stretch_body.wrist_yaw',
                'device_params': 'SE3_wrist_yaw_DW3'
            }
            }}

# ###################################33
# Baseline Nominal Params
nominal_params={
    # #################################
    #Each EOA will get expanded at runtime into its full parameter dictionary
    # Eg, supported_eoa.tool_none --> adds the wrist_yaw param dict to nominal_params
    # Add all formally supported EOA to this list
    'supported_eoa': ['eoa_wrist_dw3_tool_nil','eoa_wrist_dw3_tool_sg3', 'eoa_wrist_dw3_tool_tablet_12in'],
    'eoa_wrist_dw3_tool_nil': SE3_eoa_wrist_dw3_tool_nil,
    'eoa_wrist_dw3_tool_sg3': SE3_eoa_wrist_dw3_tool_sg3,
    'eoa_wrist_dw3_tool_tablet_12in': SE3_eoa_wrist_dw3_tool_tablet_12in,
    'arm':{
        'usb_name': '/dev/hello-motor-arm',
        'use_vel_traj': 1,
        'force_N_per_A': 55.9,  # Legacy
        'chain_pitch': 0.0167,
        'chain_sprocket_teeth': 10,
        'gr_spur': 3.875,
        'i_feedforward': 0,
        'calibration_range_bounds':[0.514, 0.525],
        'contact_models':{
            'effort_pct': {'contact_thresh_calibration_margin':10.0,'contact_thresh_max': [-90.0, 90.0]}},
        'motion':{
            'default':{
                'accel_m': 0.14,
                'vel_m': 0.14},
            'fast':{
                'accel_m': 0.3,
                'vel_m': 0.3},
            'max':{
                'accel_m': 0.4,
                'vel_m': 0.4},
            'slow':{
                'accel_m': 0.05,
                'vel_m': 0.05},
            'trajectory_max': {
                'vel_m': 0.4,
                'accel_m': 0.4},
                'vel_brakezone_factor': 0.03},
                'set_safe_velocity': 1},
    'base':{
        'usb_name_left_wheel': '/dev/hello-motor-left-wheel',
        'usb_name_right_wheel': '/dev/hello-motor-right-wheel',
        'force_N_per_A': 21.18,  # Legacy
        'gr': 3.8,
        'use_vel_traj': 0,
        'motion':{
            'default':{
                'accel_m': 0.12,
                'vel_m': 0.12},
            'fast':{
                'accel_m': 0.25,
                'vel_m': 0.25},
            'max':{
                'accel_m': 0.3,
                'vel_m': 0.3},
            'slow':{
                'accel_m': 0.06,
                'vel_m': 0.04},
            'trajectory_max': {
                'vel_r': 50.0,
                'accel_r': 30.0}},
        'contact_models':{
            'effort_pct': {
                'contact_thresh_translate_default':60.0,
                'contact_thresh_rotate_default':60.0,
                'contact_thresh_translate_max': 100.0,
                'contact_thresh_rotate_max': 100.0}},
        'sentry_max_velocity':{
            'limit_accel_m': 0.15,
            'limit_vel_m': 0.1,
            'max_arm_extension_m': 0.03,
            'max_lift_height_m': 0.3,
            'min_wrist_yaw_rad': 2.54},
        'wheel_diameter_m': 0.1016},
    'dxl_comm_errors': {
        'warn_every_s': 1.0,
        'warn_above_rate': 0.1,
        'verbose': 0},
    'end_of_arm':{
        'usb_name': '/dev/hello-dynamixel-wrist',
        'devices':{
            'wrist_yaw':{
              'py_class_name': 'WristYaw',
              'py_module_name': 'stretch_body.wrist_yaw'}},
        'use_group_sync_read': 1,
        'retry_on_comm_failure': 1,
        'baud': 115200,
        'dxl_latency_timer': 64},
    'head':{
        'usb_name': '/dev/hello-dynamixel-head',
        'use_group_sync_read': 1,
        'retry_on_comm_failure': 1,
        'dxl_latency_timer':64,
        'baud': 115200},
    'head_pan':{
        'flip_encoder_polarity': 1,
        'gr': 1.0,
        'id': 11,
        'max_voltage_limit': 15,
        'min_voltage_limit': 11,
        'motion': {
            'trajectory_vel_ctrl':1,
            'trajectory_vel_ctrl_kP':1.5,
            'default': {
                'accel': 8.0,
                'vel': 3.0},
            'fast': {
                'accel': 10.0,
                'vel': 5.0},
            'max': {
                'accel': 14.0,
                'vel': 7.0},
            'slow': {
                'accel': 4.0,
                'vel': 1.0},
            'trajectory_max': {
                'vel_r': 8.0,
                'accel_r': 16.0},
            'vel_brakezone_factor': 0.5},
        'set_safe_velocity': 1,
        'pid': [800, 200, 200],
        'pwm_homing': [-300,300],
        'pwm_limit': 885,
        'req_calibration': 0,
        'return_delay_time': 0,
        'temperature_limit': 72,
        'usb_name': '/dev/hello-dynamixel-head',
        'use_multiturn': 0,
        'use_pos_current_ctrl':0,
        'retry_on_comm_failure': 1,
        'baud': 115200,
        'enable_runstop': 1,
        'disable_torque_on_stop': 1,
        'stall_max_effort': 20.0,
        'stall_backoff': 0.017,
        'stall_max_time': 1.0,
        'stall_min_vel': 0.1,
        'range_pad_t':[50.0,-50.0],
        'range_t': [0, 3827],
        'zero_t': 1250},
    'head_tilt':{
        'flip_encoder_polarity': 1,
        'gr': 1.0,
        'id': 12,
        'max_voltage_limit': 15,
        'min_voltage_limit': 11,
        'motion': {
            'trajectory_vel_ctrl':1,
            'trajectory_vel_ctrl_kP':1.5,
            'default': {
                'accel': 8.0,
                'vel': 3.0},
            'fast': {
                'accel': 10.0,
                'vel': 5.0},
            'max': {
                'accel': 14.0,
                'vel': 7.0},
            'slow': {
                'accel': 4.0,
                'vel': 1.0},
            'trajectory_max': {
                'vel_r': 8.0,
                'accel_r': 16.0},
            'vel_brakezone_factor': 1.2},
        'set_safe_velocity': 1,
        'pid': [800,200,200],
        'pwm_homing': [-300,300],
        'pwm_limit': 885,
        'range_t': [1775,3150],
        'req_calibration': 0,
        'return_delay_time': 0,
        'temperature_limit': 72,
        'usb_name': '/dev/hello-dynamixel-head',
        'use_multiturn': 0,
        'use_pos_current_ctrl':0,
        'zero_t': 2048,
        'retry_on_comm_failure': 1,
        'baud': 115200,
        'enable_runstop': 1,
        'disable_torque_on_stop': 1,
        'stall_backoff': 0.017,
        'stall_max_effort': 20.0,
        'stall_max_time': 1.0,
        'stall_min_vel': 0.1,
        'range_pad_t': [50.0, -50.0]},
    'hello-motor-arm':{
        'gains':{
            'effort_LPF': 10.0,
            'enable_guarded_mode': 1,
            'enable_runstop': 1,
            'enable_sync_mode': 1,
            'enable_vel_watchdog':0,
            'flip_effort_polarity': 0,
            'flip_encoder_polarity': 0,
            'iMax_neg': -4.4,
            'iMax_pos': 4.4,
            'i_contact_neg': -2.0,
            'i_contact_pos': 2.0,
            'i_safety_feedforward': 0.0,
            'pKd_d': 60.0,
            'pKi_d': 0.1,
            'pKi_limit': 150,
            'pKp_d': 8.0,
            'pLPF': 60,
            'voltage_LPF':1.0,
            'phase_advance_d': 1.8,
            'pos_near_setpoint_d': 2.0,
            'safety_hold': 0,
            'safety_stiffness': 1.0,
            'vKd_d': 0,
            'vKi_d': 0.005,
            'vKi_limit': 200,
            'vKp_d': 0.2,
            'vLPF': 30,
            'vTe_d': 50,
            'vel_near_setpoint_d': 3.5,
            'vel_status_LPF': 10.0},
        'holding_torque': 1.26,
        'motion':{
            'accel': 15,
            'vel': 25},
        'rated_current': 2.8},
    'hello-motor-left-wheel':{
        'gains':{
            'effort_LPF': 2.0,
            'enable_guarded_mode': 0,
            'enable_runstop': 1,
            'enable_sync_mode': 1,
            'enable_vel_watchdog':1,
            'flip_effort_polarity': 1,
            'flip_encoder_polarity': 1,
            'iMax_neg': -4.4,
            'iMax_pos': 4.4,
            'i_contact_neg': -3.0,
            'i_contact_pos': 3.0,
            'i_safety_feedforward': 0.0,
            'pKd_d': 65.0,
            'pKi_d': 0.01,
            'pKi_limit': 50.0,
            'pKp_d': 12.0,
            'pLPF': 80.0,
            'voltage_LPF':1.0,
            'phase_advance_d': 1.8,
            'pos_near_setpoint_d': 1.0,
            'safety_hold': 0,
            'safety_stiffness': 1.0,
            'vKd_d': 0,
            'vKi_d': 0.005,
            'vKi_limit': 200,
            'vKp_d': 0.17,
            'vLPF': 30,
            'vTe_d': 50,
            'vel_near_setpoint_d': 3.5,
            'vel_status_LPF': 10.0},
        'holding_torque': 1.26,
        'motion':{
            'accel': 15,
            'vel': 25},
        'rated_current': 2.8},
    'hello-motor-lift':{
            'gains':{
            'effort_LPF': 2.0,
            'enable_guarded_mode': 1,
            'enable_runstop': 1,
            'enable_sync_mode': 1,
            'enable_vel_watchdog':0,
            'flip_effort_polarity': 1,
            'flip_encoder_polarity': 1,
            'iMax_neg': -4.40,
            'iMax_pos': 4.40,
            'i_contact_neg': -3.0,
            'i_contact_pos': 3.0,
            'i_safety_feedforward': 1.2,
            'pKd_d': 25.0,
            'pKi_d': 0.05,
            'pKi_limit': 100.0,
            'pKp_d': 6.0,
            'pLPF': 100,
            'voltage_LPF':1.0,
            'phase_advance_d': 1.8,
            'pos_near_setpoint_d': 6.0,
            'safety_hold': 1,
            'safety_stiffness': 0.0,
            'vKd_d': 0,
            'vKi_d': 0.005,
            'vKi_limit': 200,
            'vKp_d': 0.2,
            'vLPF': 30,
            'vTe_d': 50,
            'vel_near_setpoint_d': 3.5,
            'vel_status_LPF': 10.0},
        'holding_torque': 1.9,
        'motion':{
            'accel': 15,
            'vel': 12},
        'rated_current': 2.8},
    'hello-motor-right-wheel':{
        'gains':{
            'effort_LPF': 2.0,
            'enable_guarded_mode': 0,
            'enable_runstop': 1,
            'enable_sync_mode': 1,
            'enable_vel_watchdog':1,
            'flip_effort_polarity': 0,
            'flip_encoder_polarity': 0,
            'iMax_neg': -4.40,
            'iMax_pos': 4.40,
            'i_contact_neg': -3.0,
            'i_contact_pos': 3.0,
            'i_safety_feedforward': 0.0,
            'pKd_d': 65.0,
            'pKi_d': 0.01,
            'pKi_limit': 50.0,
            'pKp_d': 12.0,
            'pLPF': 80.0,
            'voltage_LPF':1.0,
            'phase_advance_d': 1.8,
            'pos_near_setpoint_d': 1.0,
            'safety_hold': 0,
            'safety_stiffness': 1.0,
            'vKd_d': 0,
            'vKi_d': 0.005,
            'vKi_limit': 200,
            'vKp_d': 0.17,
            'vLPF': 30,
            'vTe_d': 50,
            'vel_near_setpoint_d': 3.5,
            'vel_status_LPF': 10.0},
        'holding_torque': 1.26,
        'motion':{
            'accel': 15,
            'vel': 25},
        'rated_current': 2.8},
    'lift':{
        'usb_name': '/dev/hello-motor-lift',
        'use_vel_traj': 1,
        'force_N_per_A': 75.0,  # Legacy
        'calibration_range_bounds': [1.094, 1.106],
        'contact_models': {
            'effort_pct':{
                'contact_thresh_calibration_margin': 10.0,
                'contact_thresh_max': [-100, 100]}},
        'belt_pitch_m': 0.005,
          'motion':{
            'default':{
              'accel_m': 0.2,
              'vel_m': 0.11},
            'fast':{
              'accel_m': 0.25,
              'vel_m': 0.13},
            'max':{
              'accel_m': 0.3,
              'vel_m': 0.15},
            'slow':{
              'accel_m': 0.05,
              'vel_m': 0.05},
            'trajectory_max': {
              'accel_m': 0.3,
              'vel_m': 0.15},
        'vel_brakezone_factor': 0.01},
        'set_safe_velocity': 1,
          'pinion_t': 12},
    'pimu':{
      'usb_name': '/dev/hello-pimu',
      'base_fan_off': 70,
      'base_fan_on': 82,
      'max_sync_rate_hz': 80.0, #deprecated with P3
      'config':{
        'accel_LPF': 20.0,
        'bump_thresh': 20.0,
        'cliff_LPF': 10.0,
        'cliff_thresh': -50,
        'current_LPF': 10.0,
        'high_current_alert': 8.5,
        'low_voltage_alert': 10.5,
        'over_tilt_alert': 0.17,
        'stop_at_cliff': 0,
        'stop_at_high_current': 0,
        'stop_at_low_voltage': 1,
        'stop_at_runstop': 1,
        'stop_at_tilt': 0,
        'temp_LPF': 1.0,
        'voltage_LPF': 1.0}},
    'robot':{
        'rates':{
            'DXLStatusThread_Hz': 15.0,
            'NonDXLStatusThread_Hz': 25.0,
            'SystemMonitorThread_Hz': 15.0,
            'SystemMonitorThread_monitor_downrate_int': 2,
            'SystemMonitorThread_trace_downrate_int': 1,
            #'SystemMonitorThread_collision_downrate_int': 5,
            'SystemMonitorThread_sentry_downrate_int': 1,
            'SystemMonitorThread_nondxl_trajectory_downrate_int': 2},
        'tool': 'eoa_wrist_dw3_tool_sg3',
        'use_collision_manager': 0,
        'stow':{
        'arm': 0.0,
        'head_pan': 0.0,
        'head_tilt': 0.0,
        'lift': 0.23,
        'stretch_gripper': 0,
        'wrist_pitch': 0.0,
        'wrist_roll': 0.0,
        'wrist_yaw': 3.4},
        'use_monitor': 1,
        'use_trace': 0,
        'use_sentry': 1,
        'use_asyncio':1},
    'robot_monitor':{
        'monitor_base_bump_event': 1,
        'monitor_base_cliff_event': 1,
        'monitor_current': 1,
        'monitor_dynamixel_flags': 1,
        'monitor_guarded_contact': 1,
        'monitor_over_tilt_alert': 1,
        'monitor_runstop': 1,
        'monitor_voltage': 1,
        'monitor_wrist_single_tap': 1},
    'robot_sentry':{
        'base_fan_control': 1,
        'base_max_velocity': 1,
        'dynamixel_stop_on_runstop': 1,
        'stretch_gripper_overload': 1,
        'wrist_yaw_overload': 1,
        'wrist_pitch_overload': 1,
        'wrist_roll_overload': 1,
        'stepper_is_moving_filter': 1},
    'robot_trace':{
        'n_samples_per_file':100,
        'duration_limit_minutes':10.0
    },
    'robot_collision_mgmt': {
        'max_mesh_points': 48,
        'SE3': {
            'k_brake_distance': {'lift': 1.75, 'arm': 1.125, 'wrist_yaw': 0.125, 'head_pan': 0.125, 'head_tilt': 0.125},
            'collision_pairs':{'link_head_tilt_TO_link_arm_l4':{'link_pts': 'link_head_tilt', 'link_cube': 'link_arm_l4','detect_as':'pts'},
                               'link_arm_l0_TO_base_link':{'link_pts': 'link_arm_l0', 'link_cube': 'base_link','detect_as':'pts'}},

            'joints':{
                'lift': [{'motion_dir': 'pos', 'collision_pair': 'link_head_tilt_TO_link_arm_l4'},
                         {'motion_dir': 'neg', 'collision_pair': 'link_arm_l0_TO_base_link'}],
                'arm': [{'motion_dir': 'neg', 'collision_pair': 'link_arm_l0_TO_base_link'}]}
    }},
    'wacc':{
        'usb_name': '/dev/hello-wacc',
        'config': {
        'accel_LPF': 10.0,
        'accel_range_g': 4,
        'accel_single_tap_dur': 70,
        'accel_single_tap_thresh': 50,
        'ana_LPF': 10.0,
        'accel_gravity_scale': 1.0}},
    'respeaker': {'usb_name': '/dev/hello-respeaker'},
    'lidar': {'usb_name': '/dev/hello-lrf',
              'baud':115200},
    'stretch_gamepad':{
        'enable_fn_button': 0,
        'function_cmd':'',
        'press_time_span':5},
    'params':['stretch_body.robot_params_SE3_eoa']
}
