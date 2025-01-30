import rclpy
#import rospy
import datetime
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, PointField
import sensor_msgs.point_cloud2 as pc2
#import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
import cv2
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
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipe.start(config)
        
        
        self.bridge = CvBridge()
        
        self.align = rs.align(rs.stream.color)

        
        self.timer = self.create_timer(0.1, self.publish_pointcloud)

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
        #header.stamp = datetime.datetime.now()
        header.frame_id = "camera_link"

        cloud_points = []
        for point in vertices:
            cloud_points.append([point[0], point[1], point[2]])

        pointcloud_msg = create_cloud_xyz32(header, cloud_points)


        # # Convert color frame to OpenCV format
        # color_image = np.asanyarray(color_frame.get_data())
        # image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        
        # # Create PointCloud2 message
        # pointcloud_msg = PointCloud2()
        # pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        # pointcloud_msg.header.frame_id = "camera_link"
        
        # # Allocate data for the point cloud
        # width = depth_frame.get_width()
        # height = depth_frame.get_height()
        
        # # Set PointCloud2 fields (x, y, z, intensity)
        # pointcloud_msg.fields = [
        #     PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        #     PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        #     PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        # ]

        # #points = np.zeros((height, width, 4), dtype=np.float32)  # 4 columns: x, y, z, intensity
        
        # #pointcloud_msg.row_step = pointcloud_msg.width * 16  # 4 bytes per float for x, y, z, intensity
        # #pointcloud_msg.data = bytearray(pointcloud_msg.row_step * pointcloud_msg.height)
        
        # # Fill the point cloud data
        # #ptr = np.array(pointcloud_msg.data)
        # for y in range(height):
        #     for x in range(width):
        #         depth_value = depth_frame.get_distance(x, y)

        #         points[y, x, 0] = (x - width / 2) * depth_value  # x
        #         points[y, x, 1] = (y - height / 2) * depth_value  # y
        #         points[y, x, 2] = depth_value  # z
        #         points[y, x, 3] = depth_value     
        
        # pointcloud_msg.data = points.flatten().tobytes()
        # pointcloud_msg.width = width
        # pointcloud_msg.height = height
        # pointcloud_msg.row_step = width * 16


        # Publish the point cloud and the image
        self.pointcloud_pub.publish(pointcloud_msg)
        #self.color_publisher.publish(image_msg)

def main(args=None):
    print("Starting publish...")
    rclpy.init(args=args)
    node = JPointPublisher()
    rclpy.spin(node)
    node.pipe.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()