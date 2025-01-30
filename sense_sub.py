import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, PointField
from std_msgs.msg import Header
import numpy as np
#import cv2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import open3d as o3d
from sensor_msgs_py import point_cloud2 as pc2

class JPointSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_subscriber')
        self.subscription = self.create_subscription(PointCloud2, "/realsense/pointcloud", self.sub_callback, 10)


    def sub_callback(self,msg):
        self.get_logger().info(f"PointCloud2 fields: {msg.fields}")
        try:
            #print("message size: " + size(msg))
            pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        except Exception as e:
            self.get_logger().error(f"Error reading points from PointCloud2 message: {e}")
            return
        points = np.array(list(pc_data), dtype = np.float32)
        if points.size == 0:
            self.get_logger().warn("No points extracted from the PointCloud2 message.")
            return
        self.get_logger().info(f"Extracted Points (first 5): {points[:5]}")
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.visualization.draw_geometries([pcd])



def main(args=None):
    print("Searching for node")
    rclpy.init(args=args)
    sub = JPointSubscriber()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()