#!/usr/bin/env python3
# Carla仿真环境中通过ROS消息控制自驾驶车辆的姿态（Pose），但不模拟车辆的动力学（如速度、加速度等）。
# 代码接收来自ROS的车辆姿态和速度信息，并将这些信息转发给Carla中的车辆对象，进行车辆控制。
"""
Control Carla ego vehicle by using AckermannDrive messages from Mache's dbw 
"""

import sys
import logging
import numpy
import carla
import time
import rclpy
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from carla_ackermann_control import carla_control_physics as phys

from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, Pose

class NoDynamicsRelay(CompatibleNode):

    """
    Convert ackermann_drive messages to carla VehicleCommand with a PID controller
    """
#  __init__方法中定义了节点的构造函数，设置了控制循环频率、车辆的角色名称，
# 并初始化了一些关键的控制变量如车辆速度、角速度和车辆姿态。
# 同时，它还订阅了两个ROS话题，
# 分别接收车辆的速度信息（/vehicle/twist）和车辆的姿态信息（/localization/pose_estimator/pose_with_covariance）
    def __init__(self):
        """
        Constructor

        """
        super(NoDynamicsRelay, self).__init__("no_dynamics_relay")

        self.control_loop_rate = self.get_param("control_loop_rate", 0.02)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.vehicle = None
        self.ego_actor = None
        self.actor_list = []
        self.map = None
        # variables init
        self.veh_velocity = 0.0
        self.ang_velocity = 0.0
        self.veh_pose = Pose()

        # subscribe to vehicle twist
        self.twist_subscriber = self.new_subscription(
            TwistStamped,
            "/vehicle/twist",
            self.twist_updated,
            qos_profile=10
        )

        # subscribe to vehicle pose
        self.twist_subscriber = self.new_subscription(
            PoseWithCovarianceStamped,
            "/localization/pose_estimator/pose_with_covariance",
            self.pose_updated,
            qos_profile=10
        )

    def get_msg_header(self):
        """
        Get a filled ROS message header
        :return: ROS message header
        :rtype: std_msgs.msg.Header
        """
        header = Header()
        header.frame_id = "map"
        header.stamp = roscomp.ros_timestamp(sec=self.get_time(), from_sec=True)
        return header


    #回调函数twist_updated  负责更新车辆的线速度和角速度
    def twist_updated(self, veh_twist):
        """
        Get the throttle pedal from dataspeed dbw
        """
        self.veh_velocity = veh_twist.twist.linear.x
        self.ang_velocity = veh_twist.twist.angular.z

    #回调函数 pose_updated 负责更新车辆的姿态（包括位置和方向）。
    def pose_updated(self, ego_pose):
        """
        Get the throttle pedal from dataspeed dbw
        """
        self.veh_pose = ego_pose.pose.pose

    #vehicle_relay_cycle 是核心控制函数：
    # 首先，它通过调用get_waypoint函数获取车辆当前在道路上的位置，并根据道路的高度修正车辆的高度（z坐标）。
    # 然后，它将车辆的姿态中的俯仰角（Pitch）和翻滚角（Roll）移除，只保留偏航角（Yaw），
    # 并将姿态转换为Carla仿真器中使用的格式，通过set_transform将更新后的姿态应用到车辆对象上。
    def vehicle_relay_cycle(self):
        """
        Perform a vehicle control cycle and sends out CarlaEgoVehicleControl message
        """
        '''
        # Relay velocity
        velocity = carla.Vector3D()
        velocity.x = self.veh_velocity
        velocity.y = 0.0
        velocity.z = 0.0 
        self.vehicle.set_target_velocity(velocity)
        print(self.veh_velocity)
        '''


        # Assigning proper altitude
        veh_wayp = self.map.get_waypoint(self.vehicle.get_location(), project_to_road=True, lane_type=(carla.LaneType.Driving))
        self.veh_pose.position.z = veh_wayp.transform.location.z

        # Relay Pose
        # Removing roll and pitch
        veh_quat = [self.veh_pose.orientation.x, self.veh_pose.orientation.y, self.veh_pose.orientation.z, self.veh_pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(veh_quat)

        refined_quat = quaternion_from_euler(0, 0, yaw)
        self.veh_pose.orientation.x = refined_quat[0]
        self.veh_pose.orientation.y = refined_quat[1]
        self.veh_pose.orientation.z = refined_quat[2]
        self.veh_pose.orientation.w = refined_quat[3]

        ego_pose = trans.ros_pose_to_carla_transform(self.veh_pose)
        self.vehicle.set_transform(ego_pose)


    # 首先，它尝试连接Carla仿真器并获取世界对象。
    # 然后，通过遍历世界中的actor对象，找到带有role_name属性为ego_vehicle的车辆，标识为自动驾驶车辆。
    # 最后，通过调用self.new_timer设定循环频率，每隔固定时间执行一次车辆控制循环。

    def run(self):
        """

        Control loop

        :return:
        """
        # Get the client
        try:
            client = carla.Client('localhost', 2000)
            client.set_timeout(2000.0)

            world = client.get_world()
            self.map = world.get_map()
        except:
            pass
        # Set the ego vehicle
        while len(self.actor_list) == 0:
            logging.warning("Searching Actors in the world") 
            self.actor_list = world.get_actors()
            print(self.actor_list)     
            time.sleep(2.0)

    
        for actor in self.actor_list:
            if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle':
                self.vehicle = self.actor_list.find(actor.id)
                self.ego_actor = actor
                break   
        
        logging.warning("Found ego-vehicle")            
        #self.vehicle.set_enable_gravity(False)
        def loop(timer_event=None):
            self.vehicle_relay_cycle()

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """

    main function

    :return:
    """
    roscomp.init("no_dynamics_relay", args=args)

    try:
        controller = NoDynamicsRelay()
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
