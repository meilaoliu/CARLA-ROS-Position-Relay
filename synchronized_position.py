#!/usr/bin/env python3
import sys
import math
import logging
import carla
import pyproj
import rclpy
import time  # 用于控制频率
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix, Imu
import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class PositionRelay(CompatibleNode):
    """
    同步车辆的位置和IMU姿态到Carla仿真器
    """

    def __init__(self, role_name=None, update_frequency=1.0):
        """
        构造函数
        """
        super(PositionRelay, self).__init__("position_relay")

        # Carla 地图的原点坐标（从 xodr 文件中获取）
        self.origin_lat = 40.1510363262265
        self.origin_lon = 116.260639075136
        self.origin_alt = 33.146
        
        # 设置 WGS84 到 UTM 坐标转换的 pyproj 对象
        self.transformer = pyproj.Transformer.from_crs(
            "epsg:4326",  # WGS84 坐标系
            "epsg:32650",  # UTM 坐标系
            always_xy=True
        )

        self.vehicle = None
        self.ego_actor = None
        self.actor_list = []
        self.map = None
        self.veh_pose = Pose()
        self.role_name = role_name  # 动态设置 role_name 的支持

        # 订阅车辆GNSS位置信息
        self.gnss_subscriber = self.new_subscription(
            NavSatFix,
            "/sensing/gnss/ublox/nav_sat_fix",
            self.gnss_updated,
            qos_profile=10
        )

        # 订阅 IMU 数据
        self.imu_subscriber = self.new_subscription(
            Imu,
            "/sensing/imu/tamagawa/imu_raw",  # IMU 话题
            self.imu_updated,
            qos_profile=10
        )

        # 初始化姿态变量
        self.orientation = None
        self.update_frequency = 1  # 更新频率，单位 Hz
        self.last_update_time = time.time()  # 上次更新的时间
        self.yaw_offset = None  # 初始化偏差为None

    def gnss_updated(self, gnss_data):
        """
        回调函数，更新从GNSS话题接收到的车辆位置信息，并转换为Carla坐标
        """
        # 检查时间间隔，控制更新频率
        current_time = time.time()
        time_diff = current_time - self.last_update_time

        if time_diff < 1.0 / self.update_frequency:
            return  # 如果时间间隔小于设定的更新频率，直接返回

        # 更新上次执行时间
        self.last_update_time = current_time

        # GNSS 数据
        gnss_lat = gnss_data.latitude
        gnss_lon = gnss_data.longitude
        gnss_alt = gnss_data.altitude
        
        # 将 GNSS 坐标转换为 UTM 坐标
        gnss_x, gnss_y = self.transformer.transform(gnss_lon, gnss_lat)
        origin_x, origin_y = self.transformer.transform(self.origin_lon, self.origin_lat)

        # 计算相对原点的位移
        relative_x = gnss_x - origin_x
        relative_y = gnss_y - origin_y
        relative_z = gnss_alt - self.origin_alt
        
        # 更新车辆的位置
        self.veh_pose.position.x = relative_x
        self.veh_pose.position.y = relative_y
        self.veh_pose.position.z = relative_z

        # 打印相对位置
        self.get_logger().info(f"Relative Position - X: {relative_x}, Y: {relative_y}, Z: {relative_z}")

        # 每次 GNSS 更新时调用 vehicle_relay_cycle，确保位置同步
        self.vehicle_relay_cycle()

    def imu_updated(self, imu_data):
        """
        回调函数，更新从 IMU 话题接收到的车辆姿态信息（四元数）
        """
        current_time = time.time()
        time_diff = current_time - self.last_update_time

        if time_diff < 1.0 / self.update_frequency:
            return  # 如果时间间隔小于设定的更新频率，直接返回
        
        # 提取四元数并将其转换为欧拉角
        quat = [imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)

        # 如果还没有计算偏差，先计算IMU与CARLA的偏差
        if self.yaw_offset is None:
            carla_yaw = self.vehicle.get_transform().rotation.yaw * (math.pi / 180.0)  # 将CARLA的yaw角转换为弧度
            self.yaw_offset = carla_yaw - yaw  # 计算偏差
            self.get_logger().info(f"Yaw offset calculated: {self.yaw_offset}")

        # 应用偏差修正，但只在需要时使用
        yaw += self.yaw_offset
        yaw = -yaw
        yaw = yaw +19.96*(math.pi/180)
        # 仅保留航向角（yaw），将 roll 和 pitch 设置为 0
        refined_quat = quaternion_from_euler(0, 0, yaw)

        # 更新姿态到车辆
        self.veh_pose.orientation.x = refined_quat[0]
        self.veh_pose.orientation.y = refined_quat[1]
        self.veh_pose.orientation.z = refined_quat[2]
        self.veh_pose.orientation.w = refined_quat[3]

        self.get_logger().info(f"IMU Orientation updated - yaw: {yaw}")

    def vehicle_relay_cycle(self):
        """
        将车辆的位置信息同步到Carla仿真器
        """
        if self.vehicle is None:
            return

        # 调整车辆的高度以匹配道路
        veh_wayp = self.map.get_waypoint(
            self.vehicle.get_location(),
            project_to_road=True,
            lane_type=(carla.LaneType.Driving)
        )
        self.veh_pose.position.z = veh_wayp.transform.location.z  # 保持车辆在道路上

        # 将位置信息传递给Carla仿真器
        ego_pose = trans.ros_pose_to_carla_transform(self.veh_pose)
        self.vehicle.set_transform(ego_pose)

    def run(self):
        """
        主控制循环
        """
        try:
            # 连接到Carla客户端
            client = carla.Client('localhost', 2000)
            client.set_timeout(10.0)

            world = client.get_world()
            self.map = world.get_map()
        except Exception as e:
            self.get_logger().error(f"连接Carla时出错: {e}")
            return

        # 获取 ego 车辆，只执行一次
        self.actor_list = world.get_actors()

        if len(self.actor_list) == 0:
            self.get_logger().warning("未找到任何车辆或传感器。")
            return

        self.get_logger().info(f"共找到 {len(self.actor_list)} 辆车辆/传感器")

        # 查找符合条件的车辆
        for actor in self.actor_list:
            if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'hero':
                self.vehicle = actor
                self.ego_actor = actor
                break

        if not self.vehicle:
            self.get_logger().error("未找到符合条件的车辆。")
            return

        self.get_logger().info(f"找到车辆，角色名: {self.ego_actor.attributes['role_name']}")

        # 开始循环
        self.spin()


def main(args=None):
    """
    初始化ROS2节点的主函数
    """
    roscomp.init("position_relay", args=args)

    # 可以通过命令行参数或其它方式传递车辆的role_name
    role_name = None
    if len(sys.argv) > 1:
        role_name = sys.argv[1]

    try:
        controller = PositionRelay(role_name=role_name, update_frequency=10)  # 设置更新频率，10Hz
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
