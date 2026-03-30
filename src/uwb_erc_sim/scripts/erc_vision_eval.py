#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import csv
import time
import numpy as np

class ERCVisionEval(Node):
    def __init__(self):
        super().__init__('erc_vision_eval')
        
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 1)
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        self.start_time = None
        self.test_duration = 30.0  
        self.test_running = True
        
        self.file = open('luminosity_test_results.csv', 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['Time_s', 'Detected'])
        
        self.get_logger().info("Vision Node Online. Waiting for camera frame...")

    def image_cb(self, msg):
        if not self.test_running:
            return
            
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info("Camera connected! Starting 30-second recording...")
            return 
            
        elapsed = time.time() - self.start_time
        
        if elapsed >= self.test_duration:
            self.get_logger().info("30 Seconds Reached. Test Complete.")
            self.test_running = False
            rclpy.shutdown()
            return
            
        try:
            data = np.array(msg.data, dtype=np.uint8)
            cv_image = data.reshape((msg.height, msg.width, 3)).copy() 
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
            detected = 1 if (ids is not None and len(ids) > 0) else 0
            
            self.writer.writerow([round(elapsed, 2), detected])
            
            if int(elapsed * 10) % 10 == 0:
                self.get_logger().info(f"Time: {elapsed:.1f}s / {self.test_duration}s | Detected: {bool(detected)}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

def main():
    rclpy.init()
    node = ERCVisionEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()
        node.destroy_node()

if __name__ == '__main__':
    main()