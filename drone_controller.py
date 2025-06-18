import math
from pid_controller import PID, _clamp

class DroneController:
    def __init__(self, config):
        # Load config data
        self.config = config
        
        # Calculate derived constants
        self._calculate_derived_constants()

        # Initialize PID controllers
        self._initialize_pids()
        
        # Initialize state
        self.reset()

    def _calculate_derived_constants(self):
        # Physical
        self.HOVER_THRUST_TOTAL = self.config['physical_constants']['mass_kg'] * self.config['physical_constants']['gravity_mss']
        
        # Geometry
        arm_x = self.config['drone_geometry']['motor_arm_x_m']
        arm_y = self.config['drone_geometry']['motor_arm_y_m']
        self.ARM_LENGTH_M = math.sqrt(arm_x**2 + arm_y**2)
        arm_angle_rad = math.atan2(arm_y, arm_x)
        self.ARM_ANGLE_SIN = math.sin(arm_angle_rad)
        self.ARM_ANGLE_COS = math.cos(arm_angle_rad)
        
        # Motor
        thrust_const = self.config['motor_performance']['thrust_constant_n_per_rads2']
        torque_const = self.config['motor_performance']['torque_constant_nm_per_rads2']
        safety_factor = self.config['motor_performance']['factor_of_safety']
        max_speed = self.config['motor_performance']['max_motor_speed_rads']
        
        self.TORQUE_TO_THRUST_RATIO = torque_const / thrust_const
        self.MAX_MOTOR_SPEED_RADS = max_speed * safety_factor
        self.MAX_THRUST_PER_MOTOR = thrust_const * (self.MAX_MOTOR_SPEED_RADS**2)
        self.THRUST_TO_VELOCITY_FACTOR = math.sqrt(1 / thrust_const)

    def _initialize_pids(self):
        gains = self.config['pid_gains']
        self.pids = {
            'altitude':    PID(Kp=gains['z_position'][0], Ki=gains['z_position'][1], Kd=gains['z_position'][2], output_limits=(None, None), sample_time=None),
            'pos_x':       PID(Kp=gains['xy_position'][0], Ki=gains['xy_position'][1], Kd=gains['xy_position'][2], output_limits=(None, None), sample_time=None),
            'pos_y':       PID(Kp=gains['xy_position'][0], Ki=gains['xy_position'][1], Kd=gains['xy_position'][2], output_limits=(None, None), sample_time=None),
            'roll_angle':  PID(Kp=gains['roll_pitch_angle'][0], Ki=gains['roll_pitch_angle'][1], Kd=gains['roll_pitch_angle'][2], output_limits=(None, None), sample_time=None),
            'pitch_angle': PID(Kp=gains['roll_pitch_angle'][0], Ki=gains['roll_pitch_angle'][1], Kd=gains['roll_pitch_angle'][2], output_limits=(None, None), sample_time=None),
            'yaw_angle':   PID(Kp=gains['yaw_angle'][0], Ki=gains['yaw_angle'][1], Kd=gains['yaw_angle'][2], output_limits=(None, None), sample_time=None),
            'roll_rate':   PID(Kp=gains['roll_pitch_rate'][0], Ki=gains['roll_pitch_rate'][1], Kd=gains['roll_pitch_rate'][2], output_limits=(None, None), sample_time=None),
            'pitch_rate':  PID(Kp=gains['roll_pitch_rate'][0], Ki=gains['roll_pitch_rate'][1], Kd=gains['roll_pitch_rate'][2], output_limits=(None, None), sample_time=None),
            'yaw_rate':    PID(Kp=gains['yaw_rate'][0], Ki=gains['yaw_rate'][1], Kd=gains['yaw_rate'][2], output_limits=(None, None), sample_time=None),
        }

    def reset(self):
        for controller in self.pids.values():
            controller.reset()

        self.state = {
            'flight_mode': 'POSITION', 'target_altitude_m': 0.0, 'target_yaw_angle_rad': 0.0,
            'target_x_pos_m': 0.0, 'target_y_pos_m': 0.0,
            'manual_throttle_thrust': 0.0, 'mode_key_pressed_last_step': False,
            'flight_time_s': 0.0, 'initial_battery_j': -1.0, 'max_battery_j': -1.0,
            'last_gps_pos_m': None, 'last_altitude_m': None,
        }
    
    def compute_controls(self, sensor_data, input_commands, time_step_s):
        self.state['flight_time_s'] += time_step_s
        
        # Flight Mode Switching
        is_new_mode_press = input_commands['mode_switch_pressed'] and not self.state['mode_key_pressed_last_step']
        self.state['mode_key_pressed_last_step'] = input_commands['mode_switch_pressed']
        if is_new_mode_press:
            current_mode = self.state['flight_mode']
            if current_mode == 'POSITION':
                self.state['flight_mode'] = 'ANGLE'
                self.state['target_yaw_angle_rad'] = sensor_data['heading_rad']
            elif current_mode == 'ANGLE':
                self.state['flight_mode'] = 'ACRO'
                self.state['manual_throttle_thrust'] = self.HOVER_THRUST_TOTAL
            else: # ACRO to POSITION
                self.state['flight_mode'] = 'POSITION'
                self.state['target_x_pos_m'] = sensor_data['gps_values'][0]
                self.state['target_y_pos_m'] = sensor_data['gps_values'][1]
                self.state['target_altitude_m'] = sensor_data['altitude_m']
                self.state['target_yaw_angle_rad'] = sensor_data['heading_rad']

            for controller in self.pids.values(): controller.reset()
            print(f"Switched to {self.state['flight_mode']} mode.")

        # Main control logic based on flight mode
        limits = self.config['control_limits']
        
        if self.state['flight_mode'] == 'POSITION':
            self.state['target_x_pos_m'] += input_commands['roll'] * time_step_s
            self.state['target_y_pos_m'] += input_commands['pitch'] * time_step_s

            self.pids['pos_x'].setpoint = self.state['target_x_pos_m']
            target_pitch = self.pids['pos_x'](sensor_data['gps_values'][0], dt=time_step_s)
            
            self.pids['pos_y'].setpoint = self.state['target_y_pos_m']
            target_roll = self.pids['pos_y'](sensor_data['gps_values'][1], dt=time_step_s)
            
            target_pitch = _clamp(target_pitch, (-math.radians(limits['max_roll_pitch_angle_deg']), math.radians(limits['max_roll_pitch_angle_deg'])))
            target_roll = _clamp(target_roll, (-math.radians(limits['max_roll_pitch_angle_deg']), math.radians(limits['max_roll_pitch_angle_deg'])))
            
            self.pids['roll_angle'].setpoint = target_roll
            target_roll_rate = self.pids['roll_angle'](sensor_data['roll_rad'], dt=time_step_s)
            
            self.pids['pitch_angle'].setpoint = target_pitch
            target_pitch_rate = self.pids['pitch_angle'](sensor_data['pitch_rad'], dt=time_step_s)
            
            self.state['target_yaw_angle_rad'] += input_commands['yaw'] * math.radians(limits['max_yaw_rate_dps']) * time_step_s
            yaw_err = self.state['target_yaw_angle_rad'] - sensor_data['heading_rad']
            wrapped_heading = sensor_data['heading_rad'] + round(yaw_err / (2 * math.pi)) * (2 * math.pi)
            self.pids['yaw_angle'].setpoint = self.state['target_yaw_angle_rad']
            target_yaw_rate = self.pids['yaw_angle'](wrapped_heading, dt=time_step_s)
            
            self.state['target_altitude_m'] += input_commands['throttle'] * time_step_s
            self.pids['altitude'].setpoint = self.state['target_altitude_m']
            alt_thrust = self.pids['altitude'](sensor_data['altitude_m'], dt=time_step_s)
            thrust_cmd = self.HOVER_THRUST_TOTAL + alt_thrust
            control_targets = {'roll_rate': target_roll_rate, 'pitch_rate': target_pitch_rate, 'yaw_rate': target_yaw_rate, 'thrust': thrust_cmd, 'roll_angle': target_roll, 'pitch_angle': target_pitch}

        elif self.state['flight_mode'] == 'ANGLE':
            target_roll = input_commands['roll'] * math.radians(limits['max_roll_pitch_angle_deg'])
            target_pitch = input_commands['pitch'] * math.radians(limits['max_roll_pitch_angle_deg'])
            
            self.pids['roll_angle'].setpoint = target_roll
            target_roll_rate = self.pids['roll_angle'](sensor_data['roll_rad'], dt=time_step_s)
            
            self.pids['pitch_angle'].setpoint = target_pitch
            target_pitch_rate = self.pids['pitch_angle'](sensor_data['pitch_rad'], dt=time_step_s)
            
            self.state['target_yaw_angle_rad'] += input_commands['yaw'] * math.radians(limits['max_yaw_rate_dps']) * time_step_s
            yaw_err = self.state['target_yaw_angle_rad'] - sensor_data['heading_rad']
            wrapped_heading = sensor_data['heading_rad'] + round(yaw_err / (2 * math.pi)) * (2 * math.pi)
            self.pids['yaw_angle'].setpoint = self.state['target_yaw_angle_rad']
            target_yaw_rate = self.pids['yaw_angle'](wrapped_heading, dt=time_step_s)
            
            thrust_cmd = input_commands['throttle'] * (self.MAX_THRUST_PER_MOTOR * 4)
            control_targets = {'roll_rate': target_roll_rate, 'pitch_rate': target_pitch_rate, 'yaw_rate': target_yaw_rate, 'thrust': thrust_cmd, 'roll_angle': target_roll, 'pitch_angle': target_pitch}

        else: # ACRO
            target_roll_rate = input_commands['roll'] * math.radians(limits['max_acro_rate_dps'])
            target_pitch_rate = input_commands['pitch'] * math.radians(limits['max_acro_rate_dps'])
            target_yaw_rate = input_commands['yaw'] * math.radians(limits['max_yaw_rate_dps'])
            self.state['manual_throttle_thrust'] = input_commands['throttle'] * (self.MAX_THRUST_PER_MOTOR * 4)
            self.state['target_altitude_m'] = sensor_data['altitude_m']
            self.state['target_yaw_angle_rad'] = sensor_data['heading_rad']
            control_targets = {'roll_rate': target_roll_rate, 'pitch_rate': target_pitch_rate, 'yaw_rate': target_yaw_rate, 'thrust': self.state['manual_throttle_thrust'], 'roll_angle': 0.0, 'pitch_angle': 0.0}

        # Rate control (inner loop)
        max_rate = math.radians(limits['max_pitch_roll_rate_command_dps'])
        clamped_roll_rate = _clamp(control_targets['roll_rate'], (-max_rate, max_rate))
        clamped_pitch_rate = _clamp(control_targets['pitch_rate'], (-max_rate, max_rate))
        
        self.pids['roll_rate'].setpoint = clamped_roll_rate
        roll_moment = self.pids['roll_rate'](sensor_data['roll_rate_rads'], dt=time_step_s)
        
        self.pids['pitch_rate'].setpoint = clamped_pitch_rate
        pitch_moment = self.pids['pitch_rate'](sensor_data['pitch_rate_rads'], dt=time_step_s)
        
        self.pids['yaw_rate'].setpoint = control_targets['yaw_rate']
        yaw_torque = self.pids['yaw_rate'](sensor_data['yaw_rate_rads'], dt=time_step_s)
        
        # Motor Mixing
        t, r, p, y = control_targets['thrust'], roll_moment, pitch_moment, yaw_torque
        pitch_arm = self.ARM_LENGTH_M * self.ARM_ANGLE_SIN
        roll_arm = self.ARM_LENGTH_M * self.ARM_ANGLE_COS
        
        thrust_fr = (t - p / pitch_arm - r / roll_arm + y / self.TORQUE_TO_THRUST_RATIO) / 4
        thrust_rl = (t + p / pitch_arm + r / roll_arm + y / self.TORQUE_TO_THRUST_RATIO) / 4
        thrust_fl = (t - p / pitch_arm + r / roll_arm - y / self.TORQUE_TO_THRUST_RATIO) / 4
        thrust_rr = (t + p / pitch_arm - r / roll_arm - y / self.TORQUE_TO_THRUST_RATIO) / 4
        motor_thrusts = [thrust_fr, thrust_rl, thrust_fl, thrust_rr]
        
        motor_velocities = []
        for thrust in motor_thrusts:
            clamped_thrust = _clamp(thrust, (0.0, self.MAX_THRUST_PER_MOTOR))
            velocity = self.THRUST_TO_VELOCITY_FACTOR * math.sqrt(clamped_thrust)
            clamped_velocity = _clamp(velocity, (0.0, self.MAX_MOTOR_SPEED_RADS))
            motor_velocities.append(clamped_velocity)
        
        drone_targets_for_display = {'x': self.state.get('target_x_pos_m', 0.0), 'y': self.state.get('target_y_pos_m', 0.0), 'z': self.state.get('target_altitude_m', 0.0)}
            
        return motor_velocities, control_targets, drone_targets_for_display, motor_thrusts
