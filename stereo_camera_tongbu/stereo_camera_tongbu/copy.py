#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf_transformations

class StereoImageSynchronizer(Node):
    def __init__(self):
        super().__init__('stereo_image_synchronizer')
        
        self.bridge = CvBridge()
        
        # 设置QoS配置，确保可靠传输
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅通过ros_gz_bridge转换的话题
        left_image_sub = Subscriber(self, Image, '/stereo/left/image_raw', qos_profile=qos_profile)
        right_image_sub = Subscriber(self, Image, '/stereo/right/image_raw', qos_profile=qos_profile)
        left_info_sub = Subscriber(self, CameraInfo, '/stereo/left/camera_info', qos_profile=qos_profile)
        right_info_sub = Subscriber(self, CameraInfo, '/stereo/right/camera_info', qos_profile=qos_profile)

        # 创建时间同步器
        self.ts = ApproximateTimeSynchronizer(
            [left_image_sub, right_image_sub, left_info_sub, right_info_sub],
            queue_size=10,
            slop=0.1  # 允许0.1秒的时间差
        )
        self.ts.registerCallback(self.sync_callback)
        
        # 发布同步后的话题
        self.left_image_pub = self.create_publisher(
            Image, '/stereo/left/image_raw_fix', qos_profile)
        self.right_image_pub = self.create_publisher(
            Image, '/stereo/right/image_raw_fix', qos_profile)
        self.left_info_pub = self.create_publisher(
            CameraInfo, '/stereo/left/camera_info_fix', qos_profile)
        self.right_info_pub = self.create_publisher(
            CameraInfo, '/stereo/right/camera_info_fix', qos_profile)

        # 创建相机信息（内参）
        self.create_camera_info()
        
        # 创建 TF 发布器
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 存储上一次发布的时间戳
        self.last_tf_stamp = None
        
        self.get_logger().info("Stereo Image Synchronizer started")
        
    def create_camera_info(self):
        """创建相机内参信息"""
        self.left_camera_info = CameraInfo()
        self.right_camera_info = CameraInfo()
        
        # 设置header frame_id
        self.left_camera_info.header.frame_id = 'stereo_left_link'
        self.right_camera_info.header.frame_id = 'stereo_right_link'
        
        # 设置图像尺寸
        self.left_camera_info.width = 640
        self.left_camera_info.height = 480
        self.right_camera_info.width = 640
        self.right_camera_info.height = 480
        
        # 相机内参矩阵 (假设针孔相机模型)
        # 对于640x480，60度FOV的相机
        fx = 554.256  # 焦距 (像素)
        fy = 554.256
        cx = 320.0    # 主点
        cy = 240.0
        
        # 左相机内参
        self.left_camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.left_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.left_camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.left_camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        # 右相机内参（包含基线）
        baseline = 0.12  # 12cm基线
        self.right_camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.right_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.right_camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.right_camera_info.p = [fx, 0.0, cx, -fx*baseline, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
    def sync_callback(self, left_img, right_img, left_info, right_info):
        """同步回调函数"""
        # 获取当前时间
        current_time = self.get_clock().now().to_msg()
        
        # 更新所有消息的时间戳
        left_img.header.frame_id = 'stereo_left_link'
        right_img.header.frame_id = 'stereo_right_link'
        left_img.header.stamp = current_time
        right_img.header.stamp = current_time
        
        # 更新相机信息的时间戳
        self.left_camera_info.header.stamp = current_time
        self.right_camera_info.header.stamp = current_time
        
        # 发布同步后的消息
        self.left_image_pub.publish(left_img)
        self.right_image_pub.publish(right_img)
        self.left_info_pub.publish(self.left_camera_info)
        self.right_info_pub.publish(self.right_camera_info)
        
        # 发布与图像时间戳一致的 TF 变换
        self.publish_tf_with_correct_stamp(current_time)
        
        # 调试信息
        self.get_logger().debug(f"Images synchronized at time: {current_time.sec}.{current_time.nanosec}")
        
    def publish_tf_with_correct_stamp(self, stamp):
        """发布与图像时间戳一致的 TF 变换"""
        # 只在时间戳变化时发布，避免过于频繁
        if self.last_tf_stamp and \
           stamp.sec == self.last_tf_stamp.sec and \
           stamp.nanosec == self.last_tf_stamp.nanosec:
            return
            
        self.last_tf_stamp = stamp
        
        # 发布 stereo_left_link 到 stereo_right_link 的变换（基线）
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'stereo_left_link'
        t.child_frame_id = 'stereo_right_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = -0.12  # 基线12cm
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f"Published TF at time: {stamp.sec}.{stamp.nanosec}")
        
    def destroy_node(self):
        self.get_logger().info("Shutting down stereo synchronizer")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        sync_node = StereoImageSynchronizer()
        rclpy.spin(sync_node)
    except KeyboardInterrupt:
        pass
    finally:
        sync_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()