#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import csv
import time

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        # Subscribing to the D435i camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # Standard dictionary for ERC tasks
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Open CSV to log performance curves
        self.csv_file = open('performance_results.csv', mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Timestamp', 'Detected', 'Num_Markers'])
        
        self.get_logger().info("Detector Online: Saving data to performance_results.csv")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
            
            detected = 1 if ids is not None else 0
            num_markers = len(ids) if ids is not None else 0
            
            # Log the metric
            self.csv_writer.writerow([time.time(), detected, num_markers])
            self.get_logger().info(f"Detected: {detected} | Markers: {num_markers}")
            
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

def main():
    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
