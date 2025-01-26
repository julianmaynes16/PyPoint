import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, PointField
from std_msgs.msg import Header
import numpy as np
#import cv2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import open3d as o3d
class JPointSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_subscriber')
        self.subscription = self.create_subscription(PointCloud2, "/realsense/pointcloud", self.sub_callback, 10)


    def sub_callback(self,msg):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(msg)
        o3d.visualization.draw_geometried([pcd])



def main(args=None):
    print("Searching for node")
    rclpy.init(args=args)
    sub = JPointSubscriber()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()