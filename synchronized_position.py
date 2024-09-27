import sys
import math
import logging
import carla
import pyproj
import rclpy
import time
import threading
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix, Imu
import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from autoware_auto_vehicle_msgs.msg import VelocityReport
from queue import Queue, Empty

def spin_node(node):
    rclpy.spin(node)

class PositionRelay(CompatibleNode):
    """
    同步车辆的位置和IMU姿态到Carla仿真器
    """

    def __init__(self, role_name=None):
        super(PositionRelay, self).__init__("position_relay")
        self.origin_lat = 40.1510363262265
        self.origin_lon = 116.260639075136
        self.origin_alt = 33.146

        self.transformer = pyproj.Transformer.from_crs(
            "epsg:4326",   
            "epsg:32650",  
            always_xy=True
        )

        self.vehicle = None
        self.ego_actor = None
        self.actor_list = []
        self.map = None
        self.veh_pose = Pose()
        self.role_name = role_name  
        self.veh_velocity = 0.0
        self.ang_velocity = 0.0
        self.lateral_velocity = 0.0

        self.yaw_offset = 0  # 初始化偏差为None
        self.number = 0 

        # 设置订阅者并添加日志
        self.gnss_subscriber = self.new_subscription(
            NavSatFix,
            "/sensing/gnss/ublox/nav_sat_fix",
            self.gnss_updated,
            qos_profile=10
        )
        self.get_logger().info("GNSS subscription successfully created")

        self.imu_subscriber = self.new_subscription(
            Imu,
            "/sensing/imu/tamagawa/imu_raw",  
            self.imu_updated,
            qos_profile=10
        )
        self.get_logger().info("IMU subscription successfully created")

        self.velocity_subscriber = self.new_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            self.velocity_updated,
            qos_profile=10
        )
        self.get_logger().info("Velocity subscription successfully created")

    def gnss_updated(self, gnss_data):
        """
        更新GNSS数据，并将处理后的GNSS坐标加入队列
        """

        self.get_logger().info("Entered GNSS callback")

        gnss_lat = gnss_data.latitude
        gnss_lon = gnss_data.longitude
        gnss_alt = gnss_data.altitude
        
        # 将GNSS坐标转换为UTM坐标
        gnss_x, gnss_y = self.transformer.transform(gnss_lon, gnss_lat)
        origin_x, origin_y = self.transformer.transform(self.origin_lon, self.origin_lat)

        relative_x = gnss_x - origin_x
        relative_y = gnss_y - origin_y
        relative_z = gnss_alt - self.origin_alt
        
        # 更新车辆的位置
        self.veh_pose.position.x = relative_x
        self.veh_pose.position.y = relative_y
        self.veh_pose.position.z = relative_z

        self.get_logger().info(f"Relative Position - X: {relative_x}, Y: {relative_y}, Z: {relative_z}")
        

    def imu_updated(self, imu_data):
        """
        更新IMU数据，并将处理后的IMU姿态加入队列
        """
        self.vehicle_relay_cycle()  # 后同步更新车辆状态

        quat = [imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        
        # if self.yaw_offset==0 and self.number>10:
        #     carla_yaw_original = self.vehicle.get_transform().rotation.yaw 
        #     carla_yaw=carla_yaw_original* (math.pi / 180.0)  
        #     self.yaw_offset = carla_yaw - yaw  
        #     self.get_logger().info(f"yaw: {yaw}") 
        #     self.get_logger().info(f"carla_yaw_original: {carla_yaw_original}")    
        #     self.get_logger().info(f"carla_yaw: {carla_yaw}") 
        #     self.get_logger().info(f"Yaw offset calculated: {self.yaw_offset}")
        #     time.sleep(10)

        # yaw += self.yaw_offset
        
        
        
        self.get_logger().info(f"VEHICLE yaw: {yaw*180/math.pi}") 


        yaw = -yaw + 90 * (math.pi / 180) 

        #保证yaw在【-pi,pi】的范围内
        yaw =(yaw+math.pi)%(2*math.pi)-math.pi

        yaw = round(yaw, 4)

        refined_quat = quaternion_from_euler(0, 0, yaw)
  
        raid =180*(yaw/math.pi)
        self.veh_pose.orientation.x = refined_quat[0]
        self.veh_pose.orientation.y = refined_quat[1]
        self.veh_pose.orientation.z = refined_quat[2]
        self.veh_pose.orientation.w = refined_quat[3]

        self.get_logger().info(f"Yaw (rounded): {yaw},raid: {raid}")
        self.number=self.number+1
        
    def velocity_updated(self, velocity_data):
        """
        更新速度数据，并将处理后的速度加入队列
        """
        self.veh_velocity = velocity_data.longitudinal_velocity
        self.ang_velocity = velocity_data.heading_rate
        self.lateral_velocity = velocity_data.lateral_velocity

        #self.get_logger().info(f"Updated Velocity - Longitudinal: {self.veh_velocity}, Heading rate: {self.ang_velocity}")
        


    def vehicle_relay_cycle(self):
        """
        同步车辆状态，将数据从队列取出并更新车辆位置和速度
        """
        if self.vehicle is None:
            return

        veh_wayp = self.map.get_waypoint(
            self.vehicle.get_location(),
            project_to_road=True,
            lane_type=(carla.LaneType.Driving)
        )
        self.veh_pose.position.z = veh_wayp.transform.location.z  

        ego_pose = trans.ros_pose_to_carla_transform(self.veh_pose)
        self.vehicle.set_transform(ego_pose)

        transform = self.vehicle.get_transform()
        self.get_logger().info(f"vehicle yaw:{transform.rotation.yaw}")


        velocity = carla.Vector3D(self.veh_velocity, self.lateral_velocity, 0.0)
        self.vehicle.set_target_velocity(velocity)

        angular_velocity = carla.Vector3D(0.0, 0.0, self.ang_velocity)
        self.vehicle.set_target_angular_velocity(angular_velocity)

        #self.get_logger().info(f"Syncing longitudinal velocity: {self.veh_velocity}, lateral velocity: {self.lateral_velocity}, angular velocity: {self.ang_velocity}")

    def set_initial_position_and_yaw(self):
        """
        将车辆的初始位置和 yaw 角设置为特定值
        """
        initial_position = carla.Transform(
            carla.Location(x=52.846706, y=9.068787, z=156.902740),
            carla.Rotation(yaw=-20.44)
        )
        self.vehicle.set_transform(initial_position)

        # 打印更新后的位置信息
        self.get_logger().info(f"Syncing vehicle position at: Location(x=52.846706, y=9.068787, z=156.902740)")
        self.get_logger().info(f"实时 'hero' 车辆 yaw 角: -20.44 度")

    def run(self):
        try:
            client = carla.Client('localhost', 2000)
            client.set_timeout(5.0)

            world = client.get_world()
            self.map = world.get_map()

            # 启用同步模式
            original_settings = world.get_settings()
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.02  # 固定步长为0.02秒（50 Hz）
            world.apply_settings(settings)

            self.actor_list = world.get_actors()

            if len(self.actor_list) == 0:
                self.get_logger().warning("未找到任何车辆或传感器。")
                return

            self.get_logger().info(f"共找到 {len(self.actor_list)} 辆车辆/传感器")

            for actor in self.actor_list:
                if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'hero':
                    self.vehicle = actor
                    self.ego_actor = actor
                    break

            if not self.vehicle:
                self.get_logger().error("未找到符合条件的车辆。")
                return

            self.get_logger().info(f"找到车辆，角色名: {self.ego_actor.attributes['role_name']}")

            # 执行初始位置与 yaw 角更新
            #self.set_initial_position_and_yaw()

            # 启动ros 订阅回调消息 线程
            spin_thread = threading.Thread(target=spin_node, args=(self,), daemon=True)
            spin_thread.start()

            while True:
                world.tick()  # 触发同步更新
                #self.vehicle_relay_cycle()  # 每次tick后同步更新车辆状态
                time.sleep(0.02)

        finally:
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

def main(args=None):
    roscomp.init("position_relay", args=args)

    role_name = None
    if len(sys.argv) > 1:
        role_name = sys.argv[1]

    try:
        controller = PositionRelay(role_name=role_name)  
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
