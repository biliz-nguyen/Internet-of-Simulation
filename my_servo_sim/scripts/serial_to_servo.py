#!/usr/bin/env python3

import math
import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

try:
    import serial
except ImportError as exc:
    raise RuntimeError(
        "Missing pyserial. Install with: sudo apt install python3-serial"
    ) from exc


class SerialToServoNode(Node):
    """Read ESP32 serial angle lines and publish to /servo_controller/commands.

    Expected input line (from ESP32):
      - One numeric value per line, in degrees, typically in [-90, 90]
      - Example: -90, -45, 0, 45, 90
    """

    def __init__(self):
        super().__init__("serial_to_servo")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("input_unit", "deg")  # deg or rad

        # For the provided ESP32 sketch, these defaults match the sent ros_angle.
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("max_speed_deg_s", 360.0)
        self.declare_parameter("target_alpha", 1.0)
        self.declare_parameter("min_deg", -90.0)
        self.declare_parameter("max_deg", 90.0)
        self.declare_parameter("publish_deadband_deg", 0.0)
        self.declare_parameter("serial_offset_deg", 0.0)
        self.declare_parameter("no_data_warn_sec", 2.0)

        self._port = self.get_parameter("port").get_parameter_value().string_value
        self._baud = self.get_parameter("baud").get_parameter_value().integer_value
        self._input_unit = (
            self.get_parameter("input_unit").get_parameter_value().string_value.lower()
        )
        self._control_rate_hz = (
            self.get_parameter("control_rate_hz").get_parameter_value().double_value
        )
        self._max_speed_deg_s = (
            self.get_parameter("max_speed_deg_s").get_parameter_value().double_value
        )
        self._target_alpha = (
            self.get_parameter("target_alpha").get_parameter_value().double_value
        )
        self._min_deg = self.get_parameter("min_deg").get_parameter_value().double_value
        self._max_deg = self.get_parameter("max_deg").get_parameter_value().double_value
        self._publish_deadband_deg = (
            self.get_parameter("publish_deadband_deg").get_parameter_value().double_value
        )
        self._serial_offset_deg = (
            self.get_parameter("serial_offset_deg").get_parameter_value().double_value
        )
        self._no_data_warn_sec = (
            self.get_parameter("no_data_warn_sec").get_parameter_value().double_value
        )

        if self._input_unit not in ("deg", "rad"):
            raise ValueError("input_unit must be 'deg' or 'rad'")
        if self._control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be > 0")
        if self._max_speed_deg_s <= 0.0:
            raise ValueError("max_speed_deg_s must be > 0")
        if not 0.0 < self._target_alpha <= 1.0:
            raise ValueError("target_alpha must be in (0, 1]")
        if self._min_deg >= self._max_deg:
            raise ValueError("min_deg must be smaller than max_deg")
        if self._no_data_warn_sec <= 0.0:
            raise ValueError("no_data_warn_sec must be > 0")

        self._pub = self.create_publisher(Float64MultiArray, "/servo_controller/commands", 10)

        self._serial = serial.Serial(self._port, self._baud, timeout=0.1)
        self._dt = 1.0 / self._control_rate_hz
        self._max_step_rad = math.radians(self._max_speed_deg_s) * self._dt
        self._deadband_rad = math.radians(self._publish_deadband_deg)
        self._min_rad = math.radians(self._min_deg)
        self._max_rad = math.radians(self._max_deg)

        self._target_raw_rad = 0.0
        self._target_filtered_rad = 0.0
        self._command_rad = 0.0
        self._last_published_rad = float("nan")
        self._last_serial_line = ""
        self._last_rx_time = time.monotonic()
        self._tick_count = 0
        self._warned_no_data = False

        self.get_logger().info(
            f"Listening serial on {self._port} @ {self._baud}, unit={self._input_unit}"
        )
        self.get_logger().info(
            "Smoothing enabled: "
            f"rate={self._control_rate_hz:.1f}Hz, max_speed={self._max_speed_deg_s:.1f}deg/s, "
            f"alpha={self._target_alpha:.2f}, limits=[{self._min_deg:.1f},{self._max_deg:.1f}]deg, "
            f"offset={self._serial_offset_deg:.1f}deg"
        )
        self.get_logger().info("Expected line format from ESP32: -90 .. 90 (one number each line)")

        self._timer = self.create_timer(self._dt, self._control_step)

    def _control_step(self):
        self._read_serial_nonblocking()

        if time.monotonic() - self._last_rx_time > self._no_data_warn_sec:
            if not self._warned_no_data:
                self.get_logger().warn(
                    "No serial data received recently. Check ESP32 baud, cable, and Serial.println output."
                )
                self._warned_no_data = True

        # First-order filter on incoming target for extra smoothness.
        self._target_filtered_rad += self._target_alpha * (
            self._target_raw_rad - self._target_filtered_rad
        )

        # Rate-limit command to avoid abrupt position jumps.
        diff = self._target_filtered_rad - self._command_rad
        if diff > self._max_step_rad:
            self._command_rad += self._max_step_rad
        elif diff < -self._max_step_rad:
            self._command_rad -= self._max_step_rad
        else:
            self._command_rad = self._target_filtered_rad

        should_publish = (
            not math.isfinite(self._last_published_rad)
            or abs(self._command_rad - self._last_published_rad) >= self._deadband_rad
            or self._tick_count % int(max(1.0, self._control_rate_hz)) == 0
        )

        if should_publish:
            msg = Float64MultiArray()
            msg.data = [self._command_rad]
            self._pub.publish(msg)
            self._last_published_rad = self._command_rad

        self._tick_count += 1

    def _read_serial_nonblocking(self):
        latest_line = None
        while self._serial.in_waiting > 0:
            line = self._serial.readline().decode(errors="ignore").strip()
            if line:
                latest_line = line

        if latest_line is None:
            return

        value = self._parse_number(latest_line)
        if value is None:
            self.get_logger().warn(f"Ignored malformed line: '{latest_line}'")
            return

        value += self._serial_offset_deg
        angle_rad = value if self._input_unit == "rad" else math.radians(value)
        angle_rad = min(max(angle_rad, self._min_rad), self._max_rad)
        self._target_raw_rad = angle_rad
        self._last_rx_time = time.monotonic()
        self._warned_no_data = False

        if latest_line != self._last_serial_line:
            self.get_logger().info(
                f"serial='{latest_line}' -> target={angle_rad:.4f} rad ({math.degrees(angle_rad):.1f} deg)"
            )
            self._last_serial_line = latest_line

    @staticmethod
    def _parse_number(text: str):
        # Accept first numeric token from text, e.g. "angle:90" or "90".
        match = re.search(r"[-+]?\d*\.?\d+", text)
        if not match:
            return None
        return float(match.group(0))

    def destroy_node(self):
        try:
            if hasattr(self, "_serial") and self._serial.is_open:
                self._serial.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialToServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
