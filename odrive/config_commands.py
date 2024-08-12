odrv0.config.brake_resistance = 2.0
odrv0.config.dc_max_negative_current = -3.0

odrv0.axis0.motor.config.pole_pairs = 2
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4
odrv0.axis0.motor.config.requested_current_range = 25
odrv0.axis0.motor.config.current_control_bandwidth = 100
odrv0.axis0.motor.config.torque_constant = 8.27 / 62.5

odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis0.encoder.config.cpr = 6 * odrv0.axis0.motor.config.pole_pairs
odrv0.axis0.encoder.config.calib_scan_distance = 150

odrv0.axis0.encoder.config.bandwidth = 100
odrv0.axis0.controller.config.pos_gain = 1
odrv0.axis0.controller.config.vel_gain = 0.02 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
odrv0.axis0.controller.config.vel_integrator_gain = 0.1 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
odrv0.axis0.controller.config.vel_limit = 50
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

odrv0.save_configuration()
odrv0.reboot()

# Granular calibration
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION  		
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

odrv0.axis0.motor.config.pre_calibrated = True				
odrv0.axis0.encoder.config.pre_calibrated = True

odrv0.axis0.config.startup_encoder_offset_calibration = False
odrv0.axis0.config.startup_closed_loop_control = True

odrv0.save_configuration()
odrv0.reboot()

# Full calibration
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated = True

odrv0.save_configuration()
odrv0.reboot()

# Velocity control
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.input_vel = 50
odrv0.axis0.controller.input_vel = 0

# Position control
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.controller.input_pos = 50
odrv0.axis0.controller.input_pos = 0

# Axis Modes
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL # To control velocity
odrv0.axis0.requested_state = AXIS_STATE_IDLE # To release the motor

# Disable velocity limit
odrv0.axis0.controller.config.enable_vel_limit = False

# Input mode
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP


# Map PWM to velocity
odrv0.config.gpio3_mode = GPIO_MODE_PWM
odrv0.config.gpio3_pwm_mapping.min = -2
odrv0.config.gpio3_pwm_mapping.max = 2
odrv0.config.gpio3_pwm_mapping.endpoint = odrv0.axis0.controller._input_vel_property

odrv0.save_configuration()
odrv0.reboot()

# Mitigate encoder noise
# https://docs.odriverobotics.com/v/latest/troubleshooting/troubleshooting.html#encoder-noise