#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roarm_msgs.srv import MoveLineCmd

class MoveLineClient(Node):
    def __init__(self):
        super().__init__('move_line_client')
        self.cli = self.create_client(MoveLineCmd, '/move_line_cmd')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, x, y, z):
        req = MoveLineCmd.Request()
        req.x = x
        req.y = y
        req.z = z
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = MoveLineClient()

    # Example: Move to X=0.2m, Y=0.1m, Z=0.2m
    response = client.send_request(0.0, 0.0, 0.5)
    print('Response:', response)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
