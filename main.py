import sys
import json
import math
from controller import Supervisor
from gamepad_manager import GamepadManager
from display_manager import render_pfd, render_plots
from drone_controller import DroneController

def main():
    robot = Supervisor()
    time_step_ms = int(robot.getBasicTimeStep())
    time_step_s = time_step_ms / 1000.0

    # Load configuration
    with open('config.json', 'r') as f:
        config = json.load(f)

    # Initialize hardware and controllers
    gamepad = GamepadManager()
    if gamepad.joystick is None:
        print("FATAL: Controller not found. Exiting.")
        sys.exit(1)
        
    controller = DroneController(config)

    # Initialize Webots devices
    motors = [robot.getDevice(name) for name in ["front right motor", "rear left motor", "front left motor", "rear right motor"]]
    for motor in motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    sensors = {
        'imu': robot.getDevice("inertial_unit"),
        'gps': robot.getDevice("gps"),
        'gyro': robot.getDevice("gyro"),
        'compass': robot.getDevice("compass"),
        'camera': robot.getDevice("camera")
    }
    robot.batterySensorEnable(time_step_ms)
    for sensor in sensors.values():
        if hasattr(sensor, 'enable'):
            sensor.enable(time_step_ms)
            
    pfd_display = robot.getDevice("pfd_display")
    plot_display = robot.getDevice("display")

    print("STARTING IN POSITION MODE. PRESS 'A' ON CONTROLLER TO TOGGLE MODE. PRESS 'START' TO RESTART.")
    
    while robot.step(time_step_ms) != -1:
        # --- Get Inputs ---
        input_commands = gamepad.get_inputs()

        # --- Handle Supervisor Commands (Reset) ---
        if input_commands['reset_pressed']:
            robot.simulationReset()
            controller.reset()
            print("SIMULATION AND CONTROLLER STATE RESET")
            continue
        
        # --- Read Sensors ---
        roll_rad, pitch_rad, _ = sensors['imu'].getRollPitchYaw()
        compass_values = sensors['compass'].getValues()
        gps_values = sensors['gps'].getValues()
        current_altitude_m = gps_values[2]

        if controller.state['last_altitude_m'] is None or time_step_s == 0: vertical_speed_ms = 0.0
        else: vertical_speed_ms = (current_altitude_m - controller.state['last_altitude_m']) / time_step_s
        controller.state['last_altitude_m'] = current_altitude_m
        
        current_pos_m = [gps_values[0], gps_values[1]]
        if controller.state['last_gps_pos_m'] is None or time_step_s == 0: ground_speed_ms = 0.0
        else:
            dist = math.sqrt((current_pos_m[0] - controller.state['last_gps_pos_m'][0])**2 + (current_pos_m[1] - controller.state['last_gps_pos_m'][1])**2)
            ground_speed_ms = dist / time_step_s
        controller.state['last_gps_pos_m'] = current_pos_m
        
        battery_j = robot.batterySensorGetValue()
        if controller.state['initial_battery_j'] < 0:
            controller.state['initial_battery_j'] = battery_j
            controller.state['max_battery_j'] = battery_j if battery_j > 0 else 1.0
        battery_percent = max(0.0, min(1.0, battery_j / controller.state['max_battery_j'])) * 100

        sensor_data = {
            'roll_rad': roll_rad, 'pitch_rad': pitch_rad, 'altitude_m': current_altitude_m,
            'roll_rate_rads': sensors['gyro'].getValues()[0], 'pitch_rate_rads': sensors['gyro'].getValues()[1],
            'yaw_rate_rads': sensors['gyro'].getValues()[2], 'heading_rad': math.atan2(compass_values[0], compass_values[1]),
            'gps_values': gps_values, 'vertical_speed_ms': vertical_speed_ms, 'ground_speed_ms': ground_speed_ms,
            'battery_percent': battery_percent,
            'flight_time_s': controller.state['flight_time_s']
        }

        # --- Compute Controls ---
        motor_velocities, control_targets, drone_targets, motor_thrusts = controller.compute_controls(sensor_data, input_commands, time_step_s)

        # --- Set Actuators ---
        motor_directions = [-1, -1, 1, 1]
        for motor, velocity, direction in zip(motors, motor_velocities, motor_directions):
            motor.setVelocity(direction * velocity)
            
        # --- Update Displays ---
        render_pfd(pfd_display, sensors['camera'], sensor_data, controller.state['flight_mode'], motor_thrusts, controller.MAX_THRUST_PER_MOTOR, drone_targets, input_commands)
        render_plots(plot_display, control_targets, sensor_data, controller.state['target_altitude_m'], controller.state['target_yaw_angle_rad'])

if __name__ == "__main__":
    main()