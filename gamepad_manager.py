import pygame

def clamp(value, low, high):
    """Clamps a value to a given range."""
    return max(low, min(value, high))

class GamepadManager:
    """A class to handle all pygame controller logic."""
    def __init__(self):
        self.joystick = None
        pygame.init()
        if pygame.joystick.get_count() == 0:
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Controller Detected: {self.joystick.get_name()}")

        # --- Tunable Parameters ---
        self.THROTTLE_EXPO = 0.1
        self.DEADZONE = 0.1
        
        # --- Controller Mappings ---
        self.AXIS_LEFT_STICK_X = 0
        self.AXIS_LEFT_STICK_Y = 1
        self.AXIS_RIGHT_STICK_X = 2
        self.AXIS_RIGHT_STICK_Y = 3
        self.BUTTON_A = 0
        self.START = 6

    def _apply_deadzone(self, value):
        """Returns 0.0 if the value is within the deadzone threshold."""
        if abs(value) < self.DEADZONE: return 0.0
        return value

    def _apply_expo(self, value, expo_factor):
        """Applies a standard exponential curve to the input value."""
        return expo_factor * (value**3) + (1.0 - expo_factor) * value

    def get_inputs(self):
        """
        Reads the current state of the controller sticks and buttons.
        
        Returns:
            A dictionary containing the processed roll, pitch, yaw, and throttle inputs,
            or None if the controller is not available.
        """
        if self.joystick is None:
            return None

        # Pygame requires this to be called to get the latest events
        pygame.event.pump()

        # Mode 2 Drone Control Mapping
        # Get raw, deadzoned values first
        raw_throttle = -self._apply_deadzone(self.joystick.get_axis(self.AXIS_LEFT_STICK_Y))
        yaw_input    = -self._apply_deadzone(self.joystick.get_axis(self.AXIS_LEFT_STICK_X))
        pitch_input  = -self._apply_deadzone(self.joystick.get_axis(self.AXIS_RIGHT_STICK_Y))
        roll_input   =  self._apply_deadzone(self.joystick.get_axis(self.AXIS_RIGHT_STICK_X))
        
        # Apply the expo curve only to the throttle
        throttle_input = self._apply_expo(raw_throttle, self.THROTTLE_EXPO)
        
        mode_switch_pressed = self.joystick.get_button(self.BUTTON_A)
        reset_pressed = self.joystick.get_button(self.START)

        return {
            'roll': roll_input,
            'pitch': pitch_input,
            'yaw': yaw_input,
            'throttle': throttle_input,
            'mode_switch_pressed': mode_switch_pressed,
            'reset_pressed': reset_pressed,
        }