#!/usr/bin/env python3
import sys
import logging
import carla
import rclpy
from sensor_msgs.msg import Imu
import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose
import time  # 用于控制频率

class IMURelay(CompatibleNode):
    """
    只同步车辆的IMU姿态到Carla仿真器
    """

    def __init__(self, role_name=None):
        """
        构造函数
        """
        super(IMURelay, self).__init__("imu_relay")

        self.vehicle = None
        self.actor_list = []
        self.map = None
        self.role_name = role_name  # 动态设置 role_name 的支持
        self.veh_pose = Pose()

        # 订阅 IMU 数据
        self.imu_subscriber = self.new_subscription(
            Imu,
            "/sensing/imu/tamagawa/imu_raw",  # IMU 话题
            self.imu_updated,
            qos_profile=10
        )

        # 初始化姿态变量
        self.orientation = None
        # 设置频率控制相关变量
        self.update_frequency = 1 # 更新频率，单位 Hz
        self.last_update_time = time.time()  # 上次更新的时间

    def imu_updated(self, imu_data):
        """
        回调函数，更新从 IMU 话题接收到的车辆姿态信息（四元数）
        """
        # 检查时间间隔，控制更新频率
        current_time = time.time()
        time_diff = current_time - self.last_update_time

        if time_diff < 1.0 / self.update_frequency:
            return  # 如果时间间隔小于设定的更新频率，直接返回

        # 更新上次执行时间
        self.last_update_time = current_time
        self.get_logger().info("Received IMU data.")
        
        # 提取 IMU 中的四元数
        self.orientation = imu_data.orientation
        quat = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]

        # 输出原始四元数，检查是否符合预期
        self.get_logger().info(f"Original Quaternion: x={quat[0]}, y={quat[1]}, z={quat[2]}, w={quat[3]}")

        # 将四元数转换为欧拉角
        roll, pitch, yaw = euler_from_quaternion(quat)

        # 输出转换后的欧拉角，检查是否符合预期
        self.get_logger().info(f"Euler Angles: roll={roll}, pitch={pitch}, yaw={yaw}")

        # 偏航角可能需要修正
        # yaw += 180  # 根据需要调整角度，例如 CARLA 的正北方向

        # 仅保留航向角（yaw），将 roll 和 pitch 设置为 0
        refined_quat = quaternion_from_euler(0, 0, yaw)

        # 打印 IMU 姿态信息
        self.get_logger().info(f"IMU Orientation - yaw: {yaw}")

        # 同步姿态到CARLA
        #self.sync_vehicle_pose(refined_quat)


    def sync_vehicle_pose(self, refined_quat):
        """
        将IMU的姿态同步到Carla中的车辆
        """
        if self.vehicle is None:
            self.get_logger().error("Vehicle is not initialized.")
            return

        # 将姿态转换为四元数，并更新车辆姿态
        self.veh_pose.orientation.x = refined_quat[0]
        self.veh_pose.orientation.y = refined_quat[1]
        self.veh_pose.orientation.z = refined_quat[2]
        self.veh_pose.orientation.w = refined_quat[3]

        # 将姿态同步到CARLA
        ego_pose = trans.ros_pose_to_carla_transform(self.veh_pose)
        self.vehicle.set_transform(ego_pose)
        self.get_logger().info(f"Vehicle updated with yaw: {refined_quat[2]}")

    def run(self):
        """
        主控制循环
        """
        try:
            # 连接到Carla客户端
            client = carla.Client('localhost', 2000)
            client.set_timeout(10.0)

            world = client.get_world()
        except Exception as e:
            self.get_logger().error(f"连接Carla时出错: {e}")
            return

        # 获取车辆，只执行一次
        self.actor_list = world.get_actors()

        if len(self.actor_list) == 0:
            self.get_logger().warning("未找到任何车辆或传感器。")
            return

        # 查找符合条件的车辆
        for actor in self.actor_list:
            if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'hero':
                self.vehicle = actor
                break

        if not self.vehicle:
            self.get_logger().error("未找到符合条件的车辆。")
            return

        self.get_logger().info(f"找到车辆，角色名: {self.vehicle.attributes['role_name']}")

        # 开始循环
        self.spin()

def main(args=None):
    """
    初始化ROS2节点的主函数
    """
    roscomp.init("imu_relay", args=args)

    # 可以通过命令行参数传递车辆的role_name
    role_name = None
    if len(sys.argv) > 1:
        role_name = sys.argv[1]

    try:
        controller = IMURelay(role_name=role_name)
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
