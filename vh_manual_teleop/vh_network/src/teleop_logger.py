#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32
import csv
import os
from datetime import datetime

class TeleopLogger(Node):
    def __init__(self):
        super().__init__('teleop_logger')

        # --- Ficheiro CSV ---
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.expanduser("~/teleop_logs")
        os.makedirs(log_dir, exist_ok=True)
        filepath = os.path.join(log_dir, f"teleop_{timestamp}.csv")

        self.csv_file = open(filepath, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'timestamp_s', 'velocity', 'steering', 'brake',
            'gear', 'turn_signal', 'engage', 'delay_ms'
        ])
        self.get_logger().info(f"A gravar em: {filepath}")

        # --- Estado ---
        self.state = {
            'velocity': 0.0, 'steering': 0.0, 'brake': 0.0,
            'gear': 0, 'turn_signal': 1, 'engage': False, 'delay_ms': 0.0
        }

        # --- Subscriptions ---
        self.create_subscription(Float32, '/teleop/target_velocity',       lambda msg: self.update('velocity', msg.data), 10)
        self.create_subscription(Float32, '/teleop/target_steering_angle', lambda msg: self.update('steering', msg.data), 10)
        self.create_subscription(Float32, '/teleop/brake_factor',          lambda msg: self.update('brake', msg.data), 10)
        self.create_subscription(Int32,   '/teleop/gear_change',           lambda msg: self.update('gear', msg.data), 10)
        self.create_subscription(Int32,   '/teleop/turn_signal',           lambda msg: self.update('turn_signal', msg.data), 10)
        self.create_subscription(Bool,    '/teleop/engage_command',        lambda msg: self.update('engage', msg.data), 10)
        self.create_subscription(Float32, '/teleop/network_delay_ms',      lambda msg: self.update('delay_ms', msg.data), 10)

        # --- Log a 10Hz ---
        self.create_timer(0.1, self.log_state)

    def update(self, key, val):
        self.state[key] = val

    def log_state(self):
        now = self.get_clock().now().nanoseconds / 1e9
        s = self.state
        self.writer.writerow([
            f"{now:.3f}",
            f"{s['velocity']:.3f}",
            f"{s['steering']:.4f}",
            f"{s['brake']:.3f}",
            s['gear'],
            s['turn_signal'],
            s['engage'],
            f"{s['delay_ms']:.3f}"
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()