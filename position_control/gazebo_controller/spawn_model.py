# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from gazebo_msgs.srv import SpawnEntity
# from geometry_msgs.msg import Pose


# class SpawnModelClient(Node):
#     def __init__(self):
#         super().__init__('spawn_model_client')
#         self.cli = self.create_client(SpawnEntity, '/spawn_entity')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('spawn_entity 서비스 대기 중...')
#         self.req = SpawnEntity.Request()

#     def send_request(self, model_name, model_path):
#         self.req.name = model_name
#         self.req.xml = open(model_path, 'r').read()
#         self.req.robot_namespace = ''
#         self.req.reference_frame = 'world'

#         pose = Pose()
#         pose.position.x = 0.0
#         pose.position.y = 0.0
#         pose.position.z = 0.1
#         self.req.initial_pose = pose

#         future = self.cli.call_async(self.req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result()


# def main():
#     rclpy.init()
#     client = SpawnModelClient()
#     sdf_path = '/home/ciderlab-server1/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf'  # 실제 모델 경로로 바꿔주세요
#     result = client.send_request('iris', sdf_path)
#     client.get_logger().info(f'스폰 결과: {result}')
#     client.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()







#!!!!!!!!!!!!!!!don't use this code !!!!!!!!!!
#!!!!!!!!!!!!!!!don't use this code !!!!!!!!!!
#!!!!!!!!!!!!!!!don't use this code !!!!!!!!!!
#!!!!!!!!!!!!!!!don't use this code !!!!!!!!!!
#!!!!!!!!!!!!!!!don't use this code !!!!!!!!!!
