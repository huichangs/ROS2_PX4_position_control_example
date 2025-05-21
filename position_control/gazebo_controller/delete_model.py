# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from gazebo_msgs.srv import DeleteEntity


# class DeleteModelClient(Node):
#     def __init__(self):
#         super().__init__('delete_model_client')
#         self.cli = self.create_client(DeleteEntity, '/delete_entity')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('서비스 기다리는 중...')
#         self.req = DeleteEntity.Request()

#     def send_request(self, model_name):
#         self.req.name = model_name
#         future = self.cli.call_async(self.req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result()


# def main():
#     rclpy.init()
#     client = DeleteModelClient()
#     result = client.send_request('iris')  # 모델 이름
#     client.get_logger().info(f'삭제 결과: {result}')
#     client.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()





#!!!!!!!!!!!!!!!don't use this code !!!!!!!!!!
#!!!!!!!!!!!!!!!don't use this code !!!!!!!!!!
#!!!!!!!!!!!!!!!don't use this code !!!!!!!!!!
#!!!!!!!!!!!!!!!don't use this code !!!!!!!!!!
#!!!!!!!!!!!!!!!don't use this code !!!!!!!!!!

