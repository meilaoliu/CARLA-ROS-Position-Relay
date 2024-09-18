# CARLA-ROS-Position-Relay
This project is a PositionRelay system designed to synchronize vehicle positions and IMU data between CARLA and ROS environments. The system processes data from GNSS, IMU, and velocity topics, integrating real-time feedback into the CARLA vehicle simulation. The primary focus is maintaining accurate vehicle positioning and orientation in the simulation, with a role for real-time synchronization.
该项目实现了一个 PositionRelay 系统，用于在 CARLA 仿真器和 ROS 环境之间同步车辆的位置信息和 IMU 数据。该系统通过处理 GNSS、IMU 和速度话题的数据，实时反馈车辆的状态，并将其同步到 CARLA 仿真环境中，确保车辆的位置和方向的准确性。

# 功能特点
- **GNSS 位置同步：** 将 GNSS 数据转换为 UTM 坐标，实现车辆在 CARLA 中的实时位置更新。
- **IMU 数据处理：** 利用 IMU 数据进行偏航角修正，平滑处理车辆的姿态变化。
- **速度控制：** 更新车辆的纵向、横向速度以及角速度，确保仿真中车辆的动态表现。
-**同步模式支持：** 支持同步模式，在 CARLA 中保持一致的仿真时间步长。
# 系统要求
-CARLA 仿真器
-ROS 2
-Python 库：pyproj、rclpy、geometry_msgs、sensor_msgs、tf_transformations
