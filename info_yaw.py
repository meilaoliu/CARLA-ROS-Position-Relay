#!/usr/bin/env python3

import carla
import time

def main():
    try:
        # 连接到 CARLA 客户端
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # 获取仿真世界
        world = client.get_world()

        # 获取世界中的所有演员（车辆、行人、传感器等）
        actors = world.get_actors()

        # 查找名为 "hero" 的车辆
        hero_vehicle = None
        for actor in actors:
            if 'vehicle' in actor.type_id and actor.attributes.get('role_name') == 'hero':
                hero_vehicle = actor
                break

        if hero_vehicle is None:
            print("没有找到名为 'hero' 的车辆")
            return

        print(f"找到车辆 ID: {hero_vehicle.id}, 类型: {hero_vehicle.type_id}")

        # 实时获取 "hero" 车辆的 yaw 角
        while True:
            # 获取车辆的 Transform 对象
            transform = hero_vehicle.get_transform()

            # 提取 yaw 角
            yaw = transform.rotation.yaw

            # 输出当前的 yaw 角
            print(f"实时 'hero' 车辆 yaw 角: {yaw:.2f} 度")

            # 每隔一段时间打印一次
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n脚本终止")

if __name__ == '__main__':
    main()
