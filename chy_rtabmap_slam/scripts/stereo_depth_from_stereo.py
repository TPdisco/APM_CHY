#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
用左右目图像 + 视差匹配 (StereoSGBM) 计算出深度图的节点.

订阅:
    /stereo/left/image_raw    (sensor_msgs/Image)
    /stereo/right/image_raw   (sensor_msgs/Image)
    /stereo/left/camera_info  (sensor_msgs/CameraInfo)

发布:
    /stereo/depth/image_raw   (sensor_msgs/Image, 16UC1 uint16 毫米)
    /stereo/depth/camera_info (sensor_msgs/CameraInfo)

为什么选 16UC1:
    ROS 2 RViz2 的 Image/Depth 插件对 16UC1 (毫米) 兼容性最好,
    几乎不会遇到 "no image" 的渲染问题; 32FC1 (米制 float) 有时
    因 normalization 区间不匹配而无法显示.
"""

import sys
import math

import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image, CameraInfo


# ---------- 工具函数 ----------
def imgmsg_to_gray(img_msg: Image) -> np.ndarray:
    """把 sensor_msgs/Image 转成 8bit 灰度图 (h, w) uint8."""
    enc = img_msg.encoding
    if enc in ("bgr8", "rgb8"):
        arr = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
            img_msg.height, img_msg.width, 3
        )
        if enc == "rgb8":
            arr = arr[:, :, ::-1].copy()  # BGR
        return cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
    if enc in ("mono8", "8UC1"):
        return np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
            img_msg.height, img_msg.width
        )
    if enc in ("mono16", "16UC1"):
        # 16bit -> 8bit, 映射到 0-255
        a = np.frombuffer(img_msg.data, dtype=np.uint16).reshape(
            img_msg.height, img_msg.width
        )
        return cv2.normalize(a, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    if enc in ("32FC1", "float32"):
        a = np.frombuffer(img_msg.data, dtype=np.float32).reshape(
            img_msg.height, img_msg.width
        )
        return cv2.normalize(a, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    raise ValueError(f"不支持的输入编码: {enc}")


# ---------- 节点 ----------
class StereoDepthNode(Node):
    def __init__(self):
        super().__init__("stereo_depth_from_stereo")

        # 参数
        self.declare_parameter("baseline", 0.12)
        self.declare_parameter("fx_override", -1.0)
        self.declare_parameter("min_disparity", 0)
        self.declare_parameter("num_disparities", 96)
        self.declare_parameter("block_size", 9)
        self.declare_parameter("max_depth_m", 60.0)
        self.declare_parameter("out_encoding", "16UC1")
        self.declare_parameter("pre_blur", 3)
        self.declare_parameter("wls_lambda", 8000.0)
        self.declare_parameter("wls_sigma", 1.5)
        self.declare_parameter("print_stats_every_n_frames", 60)
        self.declare_parameter("left_topic", "/stereo/left/image_raw")
        self.declare_parameter("right_topic", "/stereo/right/image_raw")
        self.declare_parameter("info_topic", "/stereo/left/camera_info")
        self.declare_parameter("out_depth_topic", "/stereo/depth/image_raw")
        self.declare_parameter("out_info_topic", "/stereo/depth/camera_info")

        self.baseline = float(self.get_parameter("baseline").value)
        self.fx_override = float(self.get_parameter("fx_override").value)
        self.block_size = int(self.get_parameter("block_size").value)
        self.num_disparities = int(self.get_parameter("num_disparities").value)
        self.min_disparity = int(self.get_parameter("min_disparity").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.out_encoding = str(self.get_parameter("out_encoding").value).upper()
        if self.out_encoding not in ("16UC1", "32FC1"):
            self.get_logger().warn(
                f"out_encoding={self.out_encoding} 不支持, 回退到 16UC1"
            )
            self.out_encoding = "16UC1"
        self.pre_blur = int(self.get_parameter("pre_blur").value)
        self.wls_lambda = float(self.get_parameter("wls_lambda").value)
        self.wls_sigma = float(self.get_parameter("wls_sigma").value)
        self.print_stats_every_n_frames = int(
            self.get_parameter("print_stats_every_n_frames").value
        )

        self.left_topic = str(self.get_parameter("left_topic").value)
        self.right_topic = str(self.get_parameter("right_topic").value)
        self.info_topic = str(self.get_parameter("info_topic").value)
        self.out_depth_topic = str(self.get_parameter("out_depth_topic").value)
        self.out_info_topic = str(self.get_parameter("out_info_topic").value)

        # 相机内参缓存
        self._left_info: CameraInfo | None = None
        self._fx: float | None = None

        # 发布 / 订阅
        self.depth_pub = self.create_publisher(Image, self.out_depth_topic, 5)
        self.info_pub = self.create_publisher(CameraInfo, self.out_info_topic, 5)
        self.create_subscription(Image, self.left_topic, self._on_left, 5)
        self.create_subscription(Image, self.right_topic, self._on_right, 5)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, 5)

        # 最新图像 (简单"最新帧"配对, 仿真下时间戳一致)
        self._latest_left: Image | None = None
        self._latest_right: Image | None = None

        # 帧计数 (用于统计日志)
        self._frame_count = 0

        # SGBM
        self._init_sgbm()

        self.get_logger().info(
            f"stereo_depth_from_stereo 已启动:\n"
            f"  订阅: {self.left_topic}, {self.right_topic}, {self.info_topic}\n"
            f"  发布: {self.out_depth_topic} ({self.out_encoding})\n"
            f"  baseline={self.baseline}m, num_disparities={self.num_disparities}, "
            f"block_size={self.block_size}, wls_lambda={self.wls_lambda}"
        )

    # ---------- SGBM 初始化 ----------
    def _init_sgbm(self):
        # 保证 num_disparities 是 16 的倍数, 且 >= 16
        self.num_disparities = max(16, (int(self.num_disparities) // 16) * 16)
        # block_size 必须是奇数, 且 5 <= block_size <= 255
        self.block_size = int(self.block_size)
        if self.block_size < 5:
            self.block_size = 5
        if self.block_size % 2 == 0:
            self.block_size += 1

        self.sgbm_left = cv2.StereoSGBM_create(
            minDisparity=self.min_disparity,
            numDisparities=self.num_disparities,
            blockSize=self.block_size,
            P1=8 * self.block_size * self.block_size,
            P2=32 * self.block_size * self.block_size,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        if self.wls_lambda > 0:
            try:
                self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(
                    self.sgbm_left
                )
                self.wls_filter.setLambda(self.wls_lambda)
                self.wls_filter.setSigmaColor(self.wls_sigma)
                self.sgbm_right = cv2.ximgproc.createRightMatcher(self.sgbm_left)
            except AttributeError:
                self.get_logger().warn(
                    "你的 OpenCV 没有 ximgproc.WLS (通常是 opencv-python 版本问题). "
                    "已回退为无 WLS 模式. 如需 WLS, 请安装 opencv-contrib-python:\n"
                    "  pip install opencv-contrib-python"
                )
                self.wls_filter = None
                self.sgbm_right = None
        else:
            self.wls_filter = None
            self.sgbm_right = None

    # ---------- 回调 ----------
    def _on_info(self, msg: CameraInfo):
        self._left_info = msg
        if self.fx_override > 0:
            self._fx = float(self.fx_override)
            return
        fx = msg.k[0]
        if abs(fx) < 1e-6:
            width = msg.width if msg.width > 0 else 320
            fov = 1.047  # 约 60 度
            fx = 0.5 * width / math.tan(0.5 * fov)
            self.get_logger().warn(
                f"收到的 camera_info 中 fx 为 0, 回退估算: fx={fx:.2f}"
            )
        self._fx = float(fx)

    def _on_left(self, msg: Image):
        self._latest_left = msg
        self._try_compute()

    def _on_right(self, msg: Image):
        self._latest_right = msg
        self._try_compute()

    def _try_compute(self):
        if self._latest_left is None or self._latest_right is None:
            return
        if self._fx is None:
            # 还没拿到 camera_info, 丢弃该帧
            return

        left_msg: Image = self._latest_left
        right_msg: Image = self._latest_right
        # 消耗掉, 避免重复处理
        self._latest_left = None
        self._latest_right = None

        # --- 1. 图像解码 ---
        try:
            left_gray = imgmsg_to_gray(left_msg)
            right_gray = imgmsg_to_gray(right_msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"解码图像失败: {exc}")
            return

        # 尺寸对齐
        if left_gray.shape != right_gray.shape:
            h = min(left_gray.shape[0], right_gray.shape[0])
            w = min(left_gray.shape[1], right_gray.shape[1])
            left_gray = left_gray[:h, :w]
            right_gray = right_gray[:h, :w]

        height, width = left_gray.shape

        # --- 2. 可选高斯模糊 ---
        if self.pre_blur >= 3 and self.pre_blur % 2 == 1:
            left_gray = cv2.GaussianBlur(left_gray, (self.pre_blur, self.pre_blur), 0)
            right_gray = cv2.GaussianBlur(
                right_gray, (self.pre_blur, self.pre_blur), 0
            )

        # --- 3. 视差计算 ---
        try:
            disp_left = self.sgbm_left.compute(left_gray, right_gray)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"SGBM compute 失败: {exc}")
            return

        if self.wls_filter is not None:
            try:
                disp_right = self.sgbm_right.compute(right_gray, left_gray)
                disp_filtered = self.wls_filter.filter(
                    disp_left, left_gray, None, disp_right
                )
                # WLS 输出是 float32 (真实像素)
                disp_f = disp_filtered.astype(np.float32)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"WLS 过滤失败, 回退原始视差: {exc}")
                disp_f = disp_left.astype(np.float32) / 16.0
        else:
            # SGBM 输出是 int16, 单位是 像素 / 16
            disp_f = disp_left.astype(np.float32) / 16.0

        # --- 4. 视差 -> 深度 ---
        # depth = fx * baseline / disparity
        with np.errstate(divide="ignore", invalid="ignore"):
            depth_m = (self._fx * self.baseline) / disp_f

        # 无效值处理
        depth_m[np.isinf(depth_m)] = 0.0
        depth_m[np.isnan(depth_m)] = 0.0
        # OpenCV 对无法匹配的像素会给出 < 0 的视差, 这里统一过滤
        depth_m[disp_f < 1.0] = 0.0
        if self.max_depth_m > 0:
            depth_m[depth_m > self.max_depth_m] = 0.0
        depth_m[depth_m < 0.0] = 0.0

        # --- 5. 打包成 sensor_msgs/Image 发布 ---
        stamp = left_msg.header.stamp   # **跟随输入时间戳**, 便于 TF 对齐
        frame_id = left_msg.header.frame_id or "stereo_left_link"

        if self.out_encoding == "16UC1":
            # 毫米, 最大 65535 mm = 65.535 m
            depth_mm = np.clip(depth_m * 1000.0, 0.0, 65535.0).astype(np.uint16)
            depth_msg = Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = frame_id
            depth_msg.height = height
            depth_msg.width = width
            depth_msg.encoding = "16UC1"
            depth_msg.is_bigendian = False
            depth_msg.step = width * 2
            depth_msg.data = depth_mm.tobytes()
        else:  # 32FC1
            depth_f = np.asarray(depth_m, dtype=np.float32)
            depth_msg = Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = frame_id
            depth_msg.height = height
            depth_msg.width = width
            depth_msg.encoding = "32FC1"
            depth_msg.is_bigendian = False
            depth_msg.step = width * 4
            depth_msg.data = depth_f.tobytes()

        # 一致性检查 (防止发布无法渲染的消息)
        expected_len = depth_msg.step * depth_msg.height
        if expected_len != len(depth_msg.data):
            self.get_logger().error(
                f"深度图数据长度不一致: step*height={expected_len}, "
                f"len(data)={len(depth_msg.data)}, 跳过发布."
            )
            return

        self.depth_pub.publish(depth_msg)

        # --- 6. 发布 camera_info (复制左目) ---
        if self._left_info is not None:
            info = CameraInfo()
            info.header.stamp = stamp
            info.header.frame_id = frame_id
            info.height = height
            info.width = width
            info.distortion_model = self._left_info.distortion_model
            info.d = list(self._left_info.d)
            info.k = list(self._left_info.k)
            info.r = list(self._left_info.r)
            info.p = list(self._left_info.p)
            info.binning_x = self._left_info.binning_x
            info.binning_y = self._left_info.binning_y
            self.info_pub.publish(info)

        # --- 7. 统计日志 (每 N 帧一次) ---
        self._frame_count += 1
        if self.print_stats_every_n_frames > 0 and \
           self._frame_count % self.print_stats_every_n_frames == 0:
            total = depth_m.size
            nonzero = int(np.count_nonzero(depth_m))
            valid = depth_m[depth_m > 0.0]
            if valid.size > 0:
                self.get_logger().info(
                    f"[{self._frame_count}] 深度图: "
                    f"{depth_msg.width}x{depth_msg.height} ({depth_msg.encoding}), "
                    f"有效像素 {nonzero}/{total} "
                    f"({100.0 * nonzero / total:.1f}%), "
                    f"深度范围 {float(np.min(valid)):.2f}m - "
                    f"{float(np.max(valid)):.2f}m, "
                    f"中位数 {float(np.median(valid)):.2f}m"
                )
            else:
                self.get_logger().warn(
                    f"[{self._frame_count}] 深度图全为 0, 可能左右目图像有问题."
                )


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
