#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import csv
import time

class Automator(Node):
    def __init__(self):
        super().__init__('automator')
        self.pub = self.create_publisher(Twist, '/model/aruco_target/cmd_vel', 10)
        self.sub = self.create_subscription(Bool, '/aruco_detected', self.det_cb, 10)
        
        self.is_detected = False
        self.start_dist = 2.0  # Marker starts at 2.0m (defined in SDF)
        self.speed = 0.05      # Moves away at 5cm per second
        self.start_time = time.time()
        
        # Setup CSV
        self.file = open('performance_results.csv', 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['Distance_m', 'Detected'])
        
        self.timer = self.create_timer(0.5, self.control_loop)
        self.get_logger().info("Automator Online: Pushing marker away and logging data...")

    def det_cb(self, msg):
        self.is_detected = msg.data

    def control_loop(self):
        elapsed = time.time() - self.start_time
        current_dist = self.start_dist + (self.speed * elapsed)
        
        # Keep publishing velocity to Gazebo
        twist_msg = Twist()
        twist_msg.linear.x = self.speed
        self.pub.publish(twist_msg)
        
        # Log to file and terminal
        self.writer.writerow([round(current_dist, 3), int(self.is_detected)])
        self.get_logger().info(f"Dist: {current_dist:.2f}m | Detected: {self.is_detected}")
        
        # Stop the test once it reaches 10 meters (or adjust as needed)
        if current_dist >= 10.0:
            self.get_logger().info("Test Complete: Reached 10 meters.")
            twist_msg.linear.x = 0.0
            self.pub.publish(twist_msg)
            rclpy.shutdown()

def main():
    rclpy.init()
    node = Automator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
