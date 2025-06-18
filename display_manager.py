import math
from collections import deque

STYLE = {
    "background": 0x2B2B2B,
    "foreground": 0xFFFFFF,
    "text": 0xDCDCDC,
    "grid_light": 0x808080,
    "grid_dark": 0x404040,
    "sky": 0x44A2FF,
    "ground": 0x8B4513,
    "drone_symbol": 0xFFFF00,
    "heading_tape_bg": 0x202020,
    "tape_bg": 0x202020,
    "tape_box_bg": 0x000000,
    "mode_annunciator_bg": 0x00FF00,
    "mode_annunciator_text": 0x000000,
    "battery_green": 0x00FF00,
    "battery_yellow": 0xFFFF00,
    "battery_red": 0xFF0000,
}

GRAPH_LAYOUT = {
    "title_height": 20,
    "y_axis_width": 41,
    "x_padding": 4,
    "y_padding": 5,
    "legend_height": 15
}

def clamp(value, low, high):
    return max(low, min(value, high))

def _draw_stick_indicator(display, x, y, size, stick_x, stick_y):
    """
    Draws a single stick indicator on the HUD.
    - display: The display object to draw on.
    - x, y: Top-left coordinates of the indicator box.
    - size: The width and height of the square indicator box.
    - stick_x, stick_y: The stick input values, from -1.0 to 1.0.
    """
    center_x = x + size // 2
    center_y = y + size // 2

    # Draw semi-transparent background box
    display.setColor(STYLE["tape_box_bg"])
    display.setOpacity(0.4)
    display.fillRectangle(x, y, size, size)
    display.setOpacity(1.0)

    # Draw crosshairs
    display.setColor(STYLE["grid_light"])
    display.drawLine(x, center_y, x + size, center_y)
    display.drawLine(center_x, y, center_x, y + size)

    # Draw border
    display.setColor(STYLE["foreground"])
    display.drawRectangle(x, y, size, size)

    # Calculate and draw stick position dot
    dot_radius = 4
    padding = dot_radius + 1
    dot_x = center_x + int(stick_x * (size // 2 - padding))
    dot_y = center_y - int(stick_y * (size // 2 - padding))

    display.setColor(STYLE["drone_symbol"])
    display.fillOval(dot_x - dot_radius, dot_y - dot_radius, dot_radius * 2, dot_radius * 2)


class Graph:
    def __init__(self, display, x, y, width, height, title, is_rate_plot=False):
        self.display = display
        self.x, self.y = x, y
        self.width, self.height = width, height
        self.title = title
        self.is_symmetric_bounds = is_rate_plot
        self.plot_x = self.x + GRAPH_LAYOUT["y_axis_width"]
        self.plot_y = self.y + GRAPH_LAYOUT["title_height"]
        self.plot_width = self.width - (self.plot_x - self.x) - GRAPH_LAYOUT["x_padding"]
        self.plot_height = self.height - (self.plot_y - self.y) - GRAPH_LAYOUT["legend_height"]

    def _map_y_to_pixel(self, value, max_val, val_to_pix_ratio):
        yy_unclamped = self.plot_y + int((max_val - value) * val_to_pix_ratio)
        return clamp(yy_unclamped, self.plot_y, self.plot_y + self.plot_height)

    def _draw_series(self, history, color, max_val, val_to_pix_ratio):
        # OPTIMIZATION: Use drawLine instead of fillPolygon for performance.
        if not history or len(history) < 2: return
        self.display.setColor(color)
        num_segments = len(history) - 1
        
        # Pre-calculate all points to avoid repeated calculations in the loop
        points = [(self.plot_x + int((i / num_segments) * self.plot_width), self._map_y_to_pixel(val, max_val, val_to_pix_ratio)) for i, val in enumerate(history)]

        for i in range(num_segments):
            x1, y1 = points[i]
            x2, y2 = points[i+1]
            self.display.drawLine(x1, y1, x2, y2)

    def draw(self, target_history, actual_history, target_color, actual_color):
        self.display.setColor(STYLE["background"])
        self.display.fillRectangle(self.x, self.y, self.width, self.height)
        self.display.setColor(STYLE["foreground"])
        font_size = 10
        self.display.setFont("Arial", font_size, True)
        title_char_width = int(font_size * 0.6)
        title_width = len(self.title) * title_char_width
        centered_x = self.x + (self.width - title_width) // 2
        self.display.drawText(self.title, max(self.x + 2, centered_x), self.y + 4)

        all_values = [v for hist in (target_history, actual_history) if hist for v in hist if v is not None]
        if not all_values:
            min_val, max_val = -1.0, 1.0
        else:
            min_val, max_val = min(all_values), max(all_values)
            if self.is_symmetric_bounds:
                abs_max = max(abs(min_val), abs(max_val))
                min_val, max_val = -abs_max, abs_max
            padding = (max_val - min_val) * 0.05 if max_val > min_val else 0.1
            min_val -= padding
            max_val += padding
            if abs(max_val - min_val) < 1e-6:
                max_val += 0.1
                min_val -= 0.1

        self.display.setColor(STYLE["grid_light"])
        self.display.drawLine(self.plot_x - 1, self.plot_y, self.plot_x - 1, self.plot_y + self.plot_height)
        self.display.setFont("Arial", 8, True)
        value_range = max_val - min_val
        if abs(value_range) >= 1e-6:
            for i in range(5):
                fraction = i / 4
                value = max_val - (value_range * fraction)
                yy = self.plot_y + int(fraction * self.plot_height)
                self.display.setColor(STYLE["grid_dark"])
                self.display.drawLine(self.plot_x, yy, self.x + self.width, yy)
                self.display.setColor(STYLE["grid_light"])
                self.display.drawLine(self.plot_x - 6, yy, self.plot_x - 1, yy)
                self.display.setColor(STYLE["text"])
                self.display.drawText(f"{value:.2f}", self.x + 2, yy - 4)

        legend_y = self.y + self.height - 12
        self.display.setFont("Arial", 10, True)
        self.display.setColor(target_color)
        self.display.fillRectangle(self.x + 45, legend_y, 8, 8)
        self.display.setColor(STYLE["foreground"])
        self.display.drawText("Target", self.x + 55, legend_y)
        self.display.setColor(actual_color)
        self.display.fillRectangle(self.x + 130, legend_y, 8, 8)
        self.display.setColor(STYLE["foreground"])
        self.display.drawText("Actual", self.x + 140, legend_y)
        target_val = f"T: {target_history[-1]:.2f}" if target_history else "T: --"
        actual_val = f"A: {actual_history[-1]:.2f}" if actual_history else "A: --"
        text = f"{target_val}  {actual_val}"
        text_width = len(text) * title_char_width
        self.display.drawText(text, self.x + self.width - text_width - 5, self.y + 4)

        if abs(value_range) < 1e-6: return
        val_to_pix_ratio = self.plot_height / value_range
        
        # OPTIMIZATION: Removed the expensive filled polygon for the error area.
        
        self._draw_series(target_history, target_color, max_val, val_to_pix_ratio)
        self._draw_series(actual_history, actual_color, max_val, val_to_pix_ratio)


class TapeIndicator:
    def __init__(self, display, x, y, width, height, is_left_aligned=True, label=""):
        self.display = display
        self.x, self.y = x, y
        self.width, self.height = width, height
        self.is_left_aligned = is_left_aligned
        self.label = label
        self.center_y = self.y + self.height // 2

    def draw(self, value, major_tick_interval, pixels_per_unit):
        self.display.setColor(STYLE["tape_bg"])
        self.display.setOpacity(0.5)
        self.display.fillRectangle(self.x, self.y, self.width, self.height)
        self.display.setOpacity(1.0)

        self.display.setColor(STYLE["text"])
        self.display.setFont("Arial", 8, True)
        text_width = len(self.label) * 5
        self.display.drawText(self.label, self.x + (self.width - text_width) // 2, self.y + 2)

        box_height = 20
        box_y = self.center_y - box_height // 2

        self.display.setColor(STYLE["tape_box_bg"])
        self.display.fillRectangle(self.x, box_y, self.width, box_height)
        self.display.setColor(STYLE["foreground"])
        self.display.drawRectangle(self.x, box_y, self.width, box_height)

        text = f"{value:.1f}"
        text_width = len(text) * 6
        self.display.setFont("Arial", 10, True)
        self.display.drawText(text, self.x + (self.width - text_width) // 2, box_y + 4)

        start_val = value - (self.height / 2) / pixels_per_unit
        end_val = value + (self.height / 2) / pixels_per_unit
        start_tick = math.ceil(start_val / major_tick_interval) * major_tick_interval

        for tick_val in range(start_tick, int(end_val), major_tick_interval):
            if tick_val == 0 and self.label == "VSI": continue

            pixel_offset = (tick_val - value) * pixels_per_unit
            tick_y = self.center_y - pixel_offset

            if self.y < tick_y < self.y + self.height:
                if self.is_left_aligned:
                    tick_x1, tick_x2 = self.x + self.width - 10, self.x + self.width
                    text_x = self.x + 5
                else:
                    tick_x1, tick_x2 = self.x, self.x + 10
                    text_x = self.x + 15

                self.display.setColor(STYLE["foreground"])
                self.display.drawLine(int(tick_x1), int(tick_y), int(tick_x2), int(tick_y))
                self.display.drawText(str(tick_val), int(text_x), int(tick_y) - 4)

def render_pfd(display, camera_device, sensor_data, flight_mode, motor_thrusts, max_thrust, drone_targets, input_commands):
    if not hasattr(render_pfd, "initialized"):
        render_pfd.initialized = True
        render_pfd.PITCH_SCALE = 2.0
        render_pfd.x, render_pfd.y = 0, 0
        render_pfd.width, render_pfd.height = display.getWidth(), display.getHeight()
        render_pfd.center_x, render_pfd.center_y = render_pfd.x + render_pfd.width // 2, render_pfd.y + render_pfd.height // 2

        tape_height = 180
        tape_width = 50
        render_pfd.speed_tape = TapeIndicator(display, render_pfd.x + 15, render_pfd.center_y - tape_height // 2, tape_width, tape_height, True, "SPD")
        render_pfd.alt_tape = TapeIndicator(display, render_pfd.x + render_pfd.width - 15 - tape_width, render_pfd.center_y - tape_height // 2, tape_width, tape_height, False, "ALT")
        render_pfd.vsi_tape = TapeIndicator(display, render_pfd.x + render_pfd.width - 15 - tape_width - 25, render_pfd.center_y - 100 // 2, 20, 100, False, "VSI")

    roll, pitch, heading = sensor_data['roll_rad'], sensor_data['pitch_rad'], sensor_data['heading_rad']
    visual_roll = -roll

    camera_image_data = camera_device.getImage()
    w, h = camera_device.getWidth(), camera_device.getHeight()
    image_ref = display.imageNew(camera_image_data, 4, w, h)
    display.imagePaste(image_ref, render_pfd.x, render_pfd.y, False)
    display.imageDelete(image_ref)

    cos_roll, sin_roll = math.cos(visual_roll), math.sin(visual_roll)

    display.setColor(STYLE["foreground"])
    pitch_offset = render_pfd.PITCH_SCALE * math.degrees(pitch)
    for angle in range(-30, 31, 5):
        if angle == 0: continue
        line_y_pitched = render_pfd.center_y - (render_pfd.PITCH_SCALE * angle) + pitch_offset
        width = 40 if angle % 10 == 0 else 20

        translated_x, translated_y = (render_pfd.center_x - width / 2) - render_pfd.center_x, line_y_pitched - render_pfd.center_y
        x1 = render_pfd.center_x + (translated_x * cos_roll - translated_y * sin_roll)
        y1 = render_pfd.center_y + (translated_x * sin_roll + translated_y * cos_roll)

        translated_x, translated_y = (render_pfd.center_x + width / 2) - render_pfd.center_x, line_y_pitched - render_pfd.center_y
        x2 = render_pfd.center_x + (translated_x * cos_roll - translated_y * sin_roll)
        y2 = render_pfd.center_y + (translated_x * sin_roll + translated_y * cos_roll)

        if not (max(y1, y2) < render_pfd.y or min(y1, y2) > render_pfd.y + render_pfd.height):
            display.drawLine(int(x1), int(y1), int(x2), int(y2))

    display.setColor(STYLE["drone_symbol"])
    display.drawLine(render_pfd.center_x - 40, render_pfd.center_y, render_pfd.center_x - 15, render_pfd.center_y)
    display.drawLine(render_pfd.center_x + 15, render_pfd.center_y, render_pfd.center_x + 40, render_pfd.center_y)
    display.fillRectangle(render_pfd.center_x - 2, render_pfd.center_y - 2, 5, 5)
    display.drawLine(render_pfd.center_x, render_pfd.center_y - 5, render_pfd.center_x, render_pfd.center_y - 10)

    render_pfd.speed_tape.draw(sensor_data['ground_speed_ms'], 5, 10)
    render_pfd.alt_tape.draw(sensor_data['altitude_m'], 5, 10)
    render_pfd.vsi_tape.draw(sensor_data['vertical_speed_ms'], 2, 20)

    heading_deg = (math.degrees(heading) + 360) % 360
    tape_y, tape_height, pixels_per_degree = render_pfd.y + render_pfd.height - 50, 25, 2.0
    display.setColor(STYLE["heading_tape_bg"])
    display.setOpacity(0.7)
    display.fillRectangle(render_pfd.x + render_pfd.width // 4, tape_y - tape_height // 2, render_pfd.width // 2, tape_height)
    display.setOpacity(1.0)
    display.setColor(STYLE["foreground"])
    display.fillPolygon([render_pfd.center_x, render_pfd.center_x - 5, render_pfd.center_x + 5], [tape_y + 8, tape_y, tape_y])
    view_half_width_deg = (render_pfd.width / 4) / pixels_per_degree
    start_angle, end_angle = heading_deg - view_half_width_deg, heading_deg + view_half_width_deg
    cardinals = {0: "N", 90: "E", 180: "S", 270: "W"}
    for angle in range(int(start_angle // 10) * 10, int(end_angle // 10) * 10 + 1, 10):
        norm_angle, offset_deg = angle % 360, angle - heading_deg
        x_pos = render_pfd.center_x - int(offset_deg * pixels_per_degree)
        if not ((render_pfd.center_x - render_pfd.width//4) < x_pos < (render_pfd.center_x + render_pfd.width//4)): continue
        if norm_angle % 90 == 0: tick_height, label = 10, cardinals[norm_angle]
        elif norm_angle % 30 == 0: tick_height, label = 8, str(norm_angle // 10)
        else: tick_height, label = 4, None
        display.drawLine(x_pos, tape_y + tape_height//2 - tick_height, x_pos, tape_y + tape_height//2)
        if label: display.drawText(label, x_pos - 4, tape_y - 10)

    display.setFont("Arial", 12, True)
    mode_text = flight_mode
    text_width = len(mode_text) * 6
    text_x = render_pfd.center_x - text_width // 2
    box_padding = 10
    box_x = text_x - box_padding
    box_width = text_width + (2 * box_padding)
    display.setColor(STYLE["mode_annunciator_bg"])
    display.fillRectangle(box_x, render_pfd.y + 5, box_width, 20)
    display.setColor(STYLE["mode_annunciator_text"])
    display.drawText(mode_text, text_x, render_pfd.y + 8)
    display.setFont("Arial", 10, True)
    flight_time_s = sensor_data['flight_time_s']
    minutes, seconds = int(flight_time_s // 60), int(flight_time_s % 60)
    timer_text = f"T+ {minutes:02}:{seconds:02}"
    display.setColor(STYLE["text"])
    display.drawText(timer_text, render_pfd.x + 10, render_pfd.y + 10)
    battery_percent = sensor_data['battery_percent']
    batt_x, batt_y = render_pfd.x + render_pfd.width - 60, render_pfd.y + 10
    display.setColor(STYLE["foreground"])
    display.drawRectangle(batt_x, batt_y, 42, 15)
    display.fillRectangle(batt_x + 42, batt_y + 4, 3, 7)
    safe_battery_percent = clamp(battery_percent, 0, 100)
    fill_width = int(40 * (safe_battery_percent / 100.0))
    if battery_percent > 50: batt_color = STYLE["battery_green"]
    elif battery_percent > 20: batt_color = STYLE["battery_yellow"]
    else: batt_color = STYLE["battery_red"]
    display.setColor(batt_color)
    display.fillRectangle(batt_x + 1, batt_y + 1, fill_width, 13)
    display.setColor(STYLE["text"])
    display.drawText(f"{int(battery_percent)}%", batt_x - 30, batt_y + 3)

    display.setFont("Arial", 10, True)
    display.setColor(STYLE["foreground"])
    gps = sensor_data['gps_values']
    gps_text = f"GPS: {gps[0]:.2f}, {gps[1]:.2f}, {gps[2]:.2f}"
    display.drawText(gps_text, render_pfd.x + 5, render_pfd.y + render_pfd.height - 30)

    if flight_mode == 'POSITION':
        setpoint_text = f"GPS Setpoint: {drone_targets['x']:.2f}, {drone_targets['y']:.2f}, {drone_targets['z']:.2f}"
    else:
        setpoint_text = "GPS Setpoint: N/A, N/A, N/A"
    display.drawText(setpoint_text, render_pfd.x + 5, render_pfd.y + render_pfd.height - 15)

    bar_width, bar_height_max, spacing = 15, 60, 8
    total_width = 4 * bar_width + 3 * spacing
    start_x = render_pfd.x + render_pfd.width - total_width - 10
    bottom_y = render_pfd.y + render_pfd.height - 10
    display.setFont("Arial", 8, True)
    motor_labels = ["M1", "M2", "M3", "M4"]
    for i, thrust in enumerate(motor_thrusts):
        bar_x = start_x + i * (bar_width + spacing)
        display.setColor(STYLE["text"])
        label = motor_labels[i]
        label_width = len(label) * 4
        display.drawText(label, bar_x + (bar_width - label_width) // 2, bottom_y)
        bar_outline_y = bottom_y - bar_height_max - 12
        display.setColor(STYLE["grid_dark"])
        display.fillRectangle(bar_x, bar_outline_y, bar_width, bar_height_max)
        thrust_ratio = clamp(thrust / max_thrust, 0.0, 1.0)
        bar_fill_height = int(thrust_ratio * bar_height_max)
        if bar_fill_height > 0:
            display.setColor(STYLE["drone_symbol"])
            display.fillRectangle(bar_x, bar_outline_y + bar_height_max - bar_fill_height, bar_width, bar_fill_height)

    stick_box_size = 60
    padding_between = 20
    stick_group_width = (stick_box_size * 2) + padding_between
    group_start_x = (render_pfd.width - stick_group_width) // 2
    group_y = render_pfd.height - stick_box_size - 75
    
    _draw_stick_indicator(display,
                          group_start_x,
                          group_y,
                          stick_box_size,
                          -input_commands['yaw'],
                          input_commands['throttle'])
    
    _draw_stick_indicator(display,
                          group_start_x + stick_box_size + padding_between,
                          group_y,
                          stick_box_size,
                          input_commands['roll'],
                          input_commands['pitch'])

def render_plots(display, control_targets, sensor_data, target_altitude_m, target_yaw_angle_rad):
    if not hasattr(render_plots, "initialized"):
        render_plots.initialized = True
        widget_configs = [
            {'type': 'graph', 'label': "Altitude (m)", 'target_key': 'altitude_target', 'actual_key': 'altitude_actual', 'target_color': 0xFFFF00, 'actual_color': 0x00FF00},
            {'type': 'graph', 'label': "Yaw Angle (rad)", 'target_key': 'yaw_angle_target', 'actual_key': 'yaw_angle_actual', 'target_color': 0xFFFF00, 'actual_color': 0x00FFFF},
            {'type': 'graph', 'label': "Roll Angle (rad)", 'target_key': 'roll_angle_target', 'actual_key': 'roll_angle_actual', 'target_color': 0xFFFF00, 'actual_color': 0xFF0000},
            {'type': 'graph', 'label': "Pitch Angle (rad)", 'target_key': 'pitch_angle_target', 'actual_key': 'pitch_angle_actual', 'target_color': 0xFFFF00, 'actual_color': 0x0000FF},
            {'type': 'graph', 'label': "Roll Rate (rad/s)", 'target_key': 'roll_rate_target', 'actual_key': 'roll_rate_actual', 'target_color': 0xFFFF00, 'actual_color': 0xFF6347, 'is_rate_plot': True},
            {'type': 'graph', 'label': "Pitch Rate (rad/s)", 'target_key': 'pitch_rate_target', 'actual_key': 'pitch_rate_actual', 'target_color': 0xFFFF00, 'actual_color': 0x4169E1, 'is_rate_plot': True},
            {'type': 'graph', 'label': "Yaw Rate (rad/s)", 'target_key': 'yaw_rate_target', 'actual_key': 'yaw_rate_actual', 'target_color': 0xFFFF00, 'actual_color': 0xFF00FF, 'is_rate_plot': True},
        ]

        width, height = display.getWidth(), display.getHeight()

        render_plots.data_histories = {}
        plot_keys = {key for conf in widget_configs for key in (conf['target_key'], conf['actual_key'])}
        history_len = max(1, width)
        for key in plot_keys:
            render_plots.data_histories[key] = deque(maxlen=history_len)

        render_plots.widgets = []
        num_widgets = len(widget_configs)
        cols = 2 if num_widgets > 1 else 1
        rows = math.ceil(num_widgets / cols)
        widget_width, widget_height = width // cols, height // rows

        for i, config in enumerate(widget_configs):
            col, row = i % cols, i // cols
            x, y = col * widget_width, row * widget_height
            instance = Graph(display, x, y, widget_width, widget_height, config['label'], config.get('is_rate_plot', False))
            render_plots.widgets.append({'widget': instance, 'config': config})

    plot_values = {
        'altitude_target': target_altitude_m, 'altitude_actual': sensor_data['altitude_m'],
        'roll_angle_target': control_targets['roll_angle'], 'roll_angle_actual': sensor_data['roll_rad'],
        'pitch_angle_target': control_targets['pitch_angle'], 'pitch_angle_actual': sensor_data['pitch_rad'],
        'roll_rate_target': control_targets['roll_rate'], 'roll_rate_actual': sensor_data['roll_rate_rads'],
        'pitch_rate_target': control_targets['pitch_rate'], 'pitch_rate_actual': sensor_data['pitch_rate_rads'],
        'yaw_rate_target': control_targets['yaw_rate'], 'yaw_rate_actual': sensor_data['yaw_rate_rads'],
        'yaw_angle_target': target_yaw_angle_rad, 'yaw_angle_actual': sensor_data['heading_rad']
    }
    for name, value in plot_values.items():
        render_plots.data_histories[name].append(value)

    width, height = display.getWidth(), display.getHeight()
    display.setColor(0x000000)
    display.fillRectangle(0, 0, width, height)
    for item in render_plots.widgets:
        widget, config = item['widget'], item['config']
        target_hist = render_plots.data_histories[config['target_key']]
        actual_hist = render_plots.data_histories[config['actual_key']]
        widget.draw(target_hist, actual_hist, config['target_color'], config['actual_color'])