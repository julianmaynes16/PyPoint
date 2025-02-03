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
        #self.get_logger().info(f"PointCloud2 fields: {msg.fields}")
        #pc_data = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        # print(pc_data[0][0])
        pc_data = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
        points = np.column_stack((pc_data['x'], pc_data['y'], pc_data['z']))
        self.pcd.points = o3d.utility.Vector3dVector(points)

        self.vis.remove_geometry(self.pcd)
        self.vis.add_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()
        
    def spin(self):
        while rclpy.ok():
            self.vis.poll_events()
            self.vis.update_renderer()
            rclpy.spin_once(self)



def main(args=None):
    print("Searching for node")
    rclpy.init(args=args)
    sub = JPointSubscriber()
    sub.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()