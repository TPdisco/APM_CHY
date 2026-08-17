#!/usr/bin/env python3
"""
SLAM 地图自动保存节点
====================

功能：订阅 /map（2D 占据网格）和 /rtabmap/cloud_map（3D 点云），
     在收到 SIGTERM/SIGINT 时将最新缓存数据保存到指定目录。

设计背景：
  slam_launch_controller 通过 kill(-pgid, SIGTERM) 终止 ros2 launch 进程组时，
  rtabmap 节点会自动保存 .db 数据库，但 2D 地图和 3D 点云不会自动持久化。
  本节点作为 launch 文件中的伴随进程，持续缓存最新话题数据，
  在收到终止信号后立即写入磁盘，确保数据不丢失。

保存文件清单（{timestamp} 为保存时刻，格式 YYYYMMDD_HHMMSS）：
  - slam_map_{timestamp}.pgm  + slam_map_{timestamp}.yaml   （2D 占据网格地图）
  - slam_cloud_{timestamp}.pcd                                （3D 点云，二进制格式）
  - rtabmap.db                                                （rtabmap 自动保存）

保存目录：
  由 save_dir 参数指定，如果目录不存在则自动创建

信号处理策略：
  - SIGTERM：slam_launch_controller 发送的终止信号
  - SIGINT：用户 Ctrl+C
  - 信号处理函数仅设置标志位，不在其中调用 ROS2 API（信号安全原则）
  - 主循环检测标志位后执行保存，确保 ROS2 logger 可用
"""

import os
import signal
import struct
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2

import yaml


class MapSaveNode(Node):
    """
    地图保存节点

    持续订阅并缓存最新的 2D 地图和 3D 点云消息，
    在收到终止信号时将缓存数据写入磁盘。
    """

    def __init__(self):
        super().__init__('map_save_node')

        # ---- 声明可配置参数 ----
        # save_dir：保存目录，默认在当前工作目录下创建 saved_maps 文件夹
        self.declare_parameter('save_dir', 'saved_maps')
        # map_topic：2D 占据网格地图话题
        self.declare_parameter('map_topic', '/map')
        # cloud_topic：3D 点云话题（rtabmap 发布）
        self.declare_parameter('cloud_topic', '/rtabmap/cloud_map')

        # ---- 读取参数 ----
        # 如果 save_dir 是相对路径，则基于当前工作目录解析
        self.save_dir = os.path.abspath(self.get_parameter('save_dir').as_string())
        map_topic = self.get_parameter('map_topic').as_string()
        cloud_topic = self.get_parameter('cloud_topic').as_string()

        # ---- 数据缓存 ----
        # 只保留最新一条消息，旧消息被覆盖（节省内存）
        self.latest_map = None
        self.latest_cloud = None

        # ---- 订阅者 ----
        # QoS depth=10：保留最近 10 条消息的队列，确保不丢失
        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._map_cb, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self._cloud_cb, 10)

        # 防止重复保存的标志
        self._save_done = False

        self.get_logger().info(
            f'MapSaveNode started | save_dir={self.save_dir} '
            f'| map_topic={map_topic} | cloud_topic={cloud_topic}')

    # ------------------------------------------------------------------
    # 话题回调：仅缓存最新消息
    # ------------------------------------------------------------------
    def _map_cb(self, msg):
        """缓存最新的 2D 占据网格地图"""
        self.latest_map = msg

    def _cloud_cb(self, msg):
        """缓存最新的 3D 点云"""
        self.latest_cloud = msg

    # ------------------------------------------------------------------
    # 保存入口：协调 2D 和 3D 的保存
    # ------------------------------------------------------------------
    def save_all(self):
        """
        保存所有缓存数据到磁盘。
        使用 _save_done 标志防止重复调用（信号可能多次触发）。
        文件名包含保存时刻的时间戳，格式：YYYYMMDD_HHMMSS
        """
        if self._save_done:
            return
        self._save_done = True

        # 生成时间戳字符串，用于文件命名（精确到秒）
        self._timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # 确保保存目录存在（如果不存在则自动创建）
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f'Saving maps to {self.save_dir} ...')

        # 保存 2D 地图
        if self.latest_map is not None:
            self._save_2d_map()
        else:
            self.get_logger().warn('No 2D map data received, skip saving')

        # 保存 3D 点云
        if self.latest_cloud is not None:
            self._save_3d_cloud()
        else:
            self.get_logger().warn('No 3D cloud data received, skip saving')

        self.get_logger().info('Map saving complete')

    # ------------------------------------------------------------------
    # 2D 地图保存：PGM + YAML（ROS map_server 标准格式）
    # ------------------------------------------------------------------
    def _save_2d_map(self):
        """
        将 OccupancyGrid 消息保存为 PGM 图像 + YAML 元数据文件。

        PGM 格式说明：
          - P5 = 二进制灰度图
          - 像素值映射：-1(未知)→205, 0(自由)→254, 100(占据)→0
          - 与 ROS map_server 的保存格式完全一致

        YAML 格式说明：
          - image: PGM 文件名
          - resolution: 米/像素
          - origin: 地图左下角在世界坐标系的坐标 [x, y, yaw]
          - occupied_thresh / free_thresh: 占据/自由的概率阈值
        """
        msg = self.latest_map

        # 提取地图元数据
        w = msg.info.width          # 地图宽度（像素）
        h = msg.info.height         # 地图高度（像素）
        res = msg.info.resolution   # 分辨率（米/像素）
        ox = msg.info.origin.position.x  # 地图左下角 x 坐标
        oy = msg.info.origin.position.y  # 地图左下角 y 坐标

        # 占据值 → PGM 灰度值转换
        # OccupancyGrid.data 范围：-1(未知), 0~100(自由→占据)
        # PGM 灰度范围：0(黑)~255(白)
        #   -1(未知) → 205 (灰色，与 map_server 一致)
        #    0(自由) → 254 (接近白色)
        #  100(占据) → 0   (黑色)
        #  中间值   → 线性插值
        pgm = bytearray(w * h)
        for i, v in enumerate(msg.data):
            if v == -1:
                pgm[i] = 205
            elif v == 0:
                pgm[i] = 254
            else:
                pgm[i] = max(0, min(255, round((100 - v) * 254.0 / 100.0)))

        # 写入 PGM 二进制文件（文件名带时间戳）
        pgm_name = f'slam_map_{self._timestamp}.pgm'
        pgm_path = os.path.join(self.save_dir, pgm_name)
        with open(pgm_path, 'wb') as f:
            # PGM 头部：P5(二进制灰度) + 宽高 + 最大灰度值
            f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
            # 像素数据（行优先，从上到下）
            f.write(bytes(pgm))

        # 写入 YAML 元数据文件（文件名带时间戳，供 map_server 加载）
        yaml_name = f'slam_map_{self._timestamp}.yaml'
        yaml_path = os.path.join(self.save_dir, yaml_name)
        meta = {
            'image': pgm_name,             # 关联的 PGM 文件名（带时间戳）
            'resolution': res,              # 地图分辨率（米/像素）
            'origin': [ox, oy, 0.0],        # 地图左下角坐标 [x, y, yaw]
            'negate': 0,                    # 是否反转黑白（0=不反转）
            'occupied_thresh': 0.65,        # 占据概率阈值
            'free_thresh': 0.196,           # 自由概率阈值
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(meta, f, default_flow_style=False)

        self.get_logger().info(f'Saved 2D map: {pgm_path}')

    # ------------------------------------------------------------------
    # 3D 点云保存：二进制 PCD 格式
    # ------------------------------------------------------------------
    def _save_3d_cloud(self):
        """
        将 PointCloud2 消息保存为 PCD (Point Cloud Data) 二进制文件。

        PCD 格式说明：
          - ROS 生态广泛支持的点云存储格式
          - 二进制模式（DATA binary）写入，文件紧凑、读写快
          - 仅保存 x/y/z 三维坐标（过滤 NaN/Inf）
          - 可用 pcl_viewer、Open3D、CloudCompare 等工具打开

        PointCloud2 消息结构：
          - fields: 描述每个点的数据字段（x, y, z, rgb 等）及其偏移量
          - point_step: 一个点数据的总字节数
          - data: 原始二进制数据缓冲区
        """
        msg = self.latest_cloud

        # 查找 x/y/z 字段在点数据中的字节偏移量
        # PointCloud2 的字段布局不固定，需通过 fields 数组动态查找
        offsets = {}
        for field in msg.fields:
            if field.name in ('x', 'y', 'z'):
                offsets[field.name] = field.offset

        # 检查是否包含完整的 x/y/z 字段
        if not all(k in offsets for k in ('x', 'y', 'z')):
            self.get_logger().warn('PointCloud2 missing x/y/z fields, skip')
            return

        x_off, y_off, z_off = offsets['x'], offsets['y'], offsets['z']
        point_step = msg.point_step   # 每个点占用的字节数
        total = msg.width * msg.height  # 总点数

        pcd_path = os.path.join(self.save_dir, f'slam_cloud_{self._timestamp}.pcd')

        # 第一遍：收集所有有效点数据
        # 过滤掉包含 NaN 或 Inf 的无效点（rtabmap 可能产生此类点）
        valid_points = bytearray()
        count = 0
        for i in range(total):
            # 计算第 i 个点在 data 缓冲区中的起始位置
            base = i * point_step
            try:
                # 从二进制数据中解包 float32 (小端序) 的 x/y/z
                x = struct.unpack_from('f', msg.data, base + x_off)[0]
                y = struct.unpack_from('f', msg.data, base + y_off)[0]
                z = struct.unpack_from('f', msg.data, base + z_off)[0]
            except struct.error:
                # 数据不足，说明缓冲区已结束
                break

            # 仅保留有限值（排除 NaN 和 Inf）
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                # 将 x/y/z 打包为 3 个连续的 float32（小端序，12 字节）
                valid_points.extend(struct.pack('fff', x, y, z))
                count += 1

        # 第二遍：写入 PCD 文件（ASCII 头部 + 二进制数据）
        with open(pcd_path, 'wb') as f:
            # PCD 头部（ASCII 格式，描述数据布局）
            header = (
                '# .PCD v0.7 - Point Cloud Data file format\n'
                'VERSION 0.7\n'
                'FIELDS x y z\n'            # 字段名
                'SIZE 4 4 4\n'               # 每个字段的字节大小（float32 = 4）
                'TYPE f f f\n'               # 字段类型（f = float32）
                'COUNT 1 1 1\n'              # 每个字段的元素个数
                f'WIDTH {count}\n'            # 点云宽度（无序点云 = 总点数）
                'HEIGHT 1\n'                  # 点云高度（无序点云 = 1）
                'VIEWPOINT 0 0 0 1 0 0 0\n'  # 采集视点（tx,ty,tz,qw,qx,qy,qz）
                f'POINTS {count}\n'           # 总点数
                'DATA binary\n'               # 数据存储格式（binary = 二进制）
            )
            f.write(header.encode('ascii'))
            # 紧跟头部写入二进制点数据
            f.write(bytes(valid_points))

        self.get_logger().info(f'Saved 3D cloud: {pcd_path} ({count} points)')


def main():
    """
    主函数：初始化节点，注册信号处理，运行事件循环。

    信号处理策略：
      1. SIGTERM（slam_launch_controller 发送的终止信号）：
         信号处理函数仅设置 save_flag = True
      2. 主循环中检测 save_flag，执行保存操作
      3. 保存完成后 break 退出循环

    为什么不在信号处理函数中直接保存？
      - Python 信号处理函数在主线程中执行，但 ROS2 的 logger 和文件 I/O
        不是信号安全的（signal-safe）
      - 在信号处理函数中调用 ROS2 API 可能导致死锁或数据损坏
      - 因此采用"标志位 + 主循环检测"的异步模式
    """
    rclpy.init()
    node = MapSaveNode()

    # 使用列表包装标志位，使信号处理函数（闭包）可以修改外部变量
    # Python 2 中不可变类型的闭包赋值需要 nonlocal，列表包装更兼容
    save_flag = [False]

    def on_sigterm(signum, frame):
        """
        SIGTERM 信号处理函数
        仅设置标志位，不在其中执行保存操作（信号安全原则）
        """
        save_flag[0] = True

    # 注册 SIGTERM 处理函数
    signal.signal(signal.SIGTERM, on_sigterm)

    try:
        # 主事件循环：spin_once + 信号标志检测
        while rclpy.ok():
            # spin_once 处理一次回调（订阅消息），timeout_sec=0.5 避免忙等
            rclpy.spin_once(node, timeout_sec=0.5)

            # 检测 SIGTERM 标志
            if save_flag[0]:
                node.get_logger().info('SIGTERM received, saving maps...')
                node.save_all()
                break

    except KeyboardInterrupt:
        # Ctrl+C 也可触发保存（开发调试时使用）
        node.get_logger().info('KeyboardInterrupt, saving maps...')
        node.save_all()

    finally:
        # 清理 ROS2 资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
