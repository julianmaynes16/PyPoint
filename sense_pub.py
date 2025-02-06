import rclpy
#import rospy
import datetime
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, PointField
import sensor_msgs_py.point_cloud2 as pc2
#import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
import cv2
from geometry_msgs.msg import TransformStamped
import tf2_ros
from cv_bridge import CvBridge
import pyrealsense2 as rs

class JPointPublisher(Node):
    def __init__(self):
        super().__init__('realsense_pointcloud_publisher')
        
        # Create the publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'realsense/pointcloud', 10)
        self.color_publisher = self.create_publisher(Image, 'realsense/color_image', 10)
        
        # Start the RealSense pipeline
        self.pipe = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        self.pipe.start(config)
        
        
        self.bridge = CvBridge()
        
        self.align = rs.align(rs.stream.color)

        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self, )

        self.timer = self.create_timer(1, self.publish_pointcloud)

    def publish_pointcloud(self):
        
        frames = self.pipe.wait_for_frames()
        aligned_frames = self.align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        pc = rs.pointcloud()
        points = pc.calculate(depth_frame)
        pc.map_to(color_frame)
        vertices = np.asanyarray(points.get_vertices())

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link"

        cloud_points = []
        for point in vertices:
            cloud_points.append([point[0], point[1], point[2]])

        pointcloud_msg = pc2.create_cloud_xyz32(header, cloud_points)


        # Publish the point cloud and the image
        self.pointcloud_pub.publish(pointcloud_msg)
        #self.color_publisher.publish(image_msg)

        self.broadcast_transform()
        
    def broadcast_transform(self):
        # Example transform broadcasting from "world" to "camera_link"
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "camera_link"

        # Assuming the camera is at the origin, but you can change this if needed
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    print("Starting publish...")
    rclpy.init(args=args)
    node = JPointPublisher()
    rclpy.spin(node)
    node.pipe.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()