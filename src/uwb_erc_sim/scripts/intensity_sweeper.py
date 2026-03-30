#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class IntensitySweeper(Node):
    def __init__(self, step=5.0, max_lux=100.0):
        super().__init__('intensity_sweeper')
        # Topic bridging to Gazebo Harmonic's light system
        self.publisher_ = self.create_publisher(Float64, '/camera/light/spotlight/intensity', 10)
        self.step = step
        self.max_lux = max_lux
        self.current_lux = 0.0
        
        # Step the light up every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info(f"Starting Intensity Sweep: 0 to {max_lux} lux in steps of {step}")

    def timer_callback(self):
        if self.current_lux <= self.max_lux:
            msg = Float64()
            msg.data = float(self.current_lux)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Current Spotlight Intensity: {self.current_lux} lux")
            self.current_lux += self.step
        else:
            self.get_logger().info("Sweep Complete. Maximum intensity reached.")
            self.timer.cancel()

def main():
    rclpy.init()
    node = IntensitySweeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
