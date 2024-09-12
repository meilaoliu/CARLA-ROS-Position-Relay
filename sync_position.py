#!/usr/bin/env python3
import sys
import logging
import carla
import pyproj
import rclpy
import time  # 用于控制频率
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix
import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

class PositionRelay(CompatibleNode):
    """
    只同步车辆的位置到Carla仿真器
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

        # 设置频率控制相关变量
        self.update_frequency = 50 # 更新频率，单位 Hz
        self.last_update_time = time.time()  # 上次更新的时间

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
        print(f"Relative Position - X: {relative_x}, Y: {relative_y}, Z: {relative_z}")

        # 每次 GNSS 更新时调用 vehicle_relay_cycle，确保位置同步
        self.vehicle_relay_cycle()

    def vehicle_relay_cycle(self):
        """
        将车辆的位置信息同步到Carla仿真器
        """
        if self.vehicle is None:
            return

        # 添加调试输出，确认每次进入循环时是否有输出
        print(f"Syncing vehicle position at: {self.vehicle.get_location()}")
        logging.info(f"Syncing vehicle position at: {self.vehicle.get_location()}")

        # 根据当前道路调整车辆的高度
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
            logging.error(f"连接Carla时出错: {e}")
            return

        # 设置 ego 车辆，只执行一次
        self.actor_list = world.get_actors()

        if len(self.actor_list) == 0:
            logging.warning("未找到任何车辆或传感器。")
            return

        logging.info(f"共找到 {len(self.actor_list)} 辆车辆/传感器")

        # 遍历车辆列表，查找符合条件的 hero 车辆，只执行一次
        for actor in self.actor_list:
            print(f"Vehicle ID: {actor.id}, Type ID: {actor.type_id}, Role Name: {actor.attributes.get('role_name', 'N/A')}")
            if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'hero':
                self.vehicle = actor  # 直接使用 actor 而不是 find
                self.ego_actor = actor
                break

        if not self.vehicle:
            logging.error("未找到符合条件的车辆。")
            return

        logging.info(f"找到车辆，角色名: {self.ego_actor.attributes['role_name']}")

        # 开始循环
        print("Entering spin function...")
        self.spin()


def main(args=None):
    """
    初始化ROS2节点的主函数
    """
    roscomp.init("position_relay", args=args)

    # 可以通过命令行参数或其它方式传递车辆的role_name
    role_name = None
    if len(sys.argv) > 1:
        role_name = sys.argv[1]  # 从命令行传递车辆的角色名

    try:
        controller = PositionRelay(role_name=role_name, update_frequency=10)  # 设置更新频率，10Hz
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
