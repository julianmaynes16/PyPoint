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
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.pcd = o3d.geometry.PointCloud()

    def sub_callback(self,msg):
        self.get_logger().info(f"PointCloud2 fields: {msg.fields}")
        pc_data = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        # print(pc_data[0][0])
        pc_data = np.array(pc_data, dtype=np.float32)
        self.vis.remove_geometry(self.pcd)
        self.pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_data))
        self.vis.add_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()
        
        



def main(args=None):
    print("Searching for node")
    rclpy.init(args=args)
    sub = JPointSubscriber()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()