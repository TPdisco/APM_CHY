#!/usr/bin/env python3
"""
USB摄像头持续录制节点

用法:
  ros2 launch copter_run_cpp camera_record.launch.py

参数:
  camera_id: USB摄像头设备ID (默认0)
  output_dir: 视频保存目录 (默认 ~/camera_recordings)
  fps: 录制帧率 (默认30)
  width: 分辨率宽度 (默认640)
  height: 分辨率高度 (默认480)
"""

import os
import time
import cv2
import rclpy
from rclpy.node import Node


class CameraRecorder(Node):
    def __init__(self):
        super().__init__('camera_recorder')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('output_dir', os.path.expanduser('~/camera_recordings'))
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        self.camera_id = self.get_parameter('camera_id').value
        self.output_dir = self.get_parameter('output_dir').value
        self.fps = self.get_parameter('fps').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info(f'打开摄像头 ID={self.camera_id}')
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开摄像头 {self.camera_id}')
            raise RuntimeError(f'Camera {self.camera_id} not available')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        real_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        real_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f'摄像头分辨率: {real_w}x{real_h}')

        self._init_writer()

        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(f'开始录制, 保存到: {self.video_path}')

    def _init_writer(self):
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.video_path = os.path.join(self.output_dir, f'camera_{timestamp}.mp4')
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(
            self.video_path, fourcc, self.fps,
            (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
             int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('读取帧失败')
            return

        self.writer.write(frame)

    def destroy_node(self):
        self.get_logger().info(f'录制结束, 文件: {self.video_path}')
        if hasattr(self, 'writer'):
            self.writer.release()
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()