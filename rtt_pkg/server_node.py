#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.clock import ROSClock

from custom_interfaces2.action import Rtt

class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')

        self._action_server = ActionServer(
            self,
            Rtt,
            'my_action',
            execute_callback = self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback
        )

    def goal_callback(self, goal_request):
        self.get_logger().info('Received a goal request')
        return rclpy.action.server.GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info('Accepted a goal request')
        return goal_handle.execute()#를 제거합니다.
        

    def execute_callback(self, goal_handle):
        # goal_handle.succeed()
        self.get_logger().info('Received request time: {0}'.format(goal_handle.request.request_time))
        
        response_time = ROSClock().now()
        self.get_logger().info('Sending response time: {0}'.format(response_time.to_msg()))

        # Create a result message and set the response time
        
        result = Rtt.Result()
        result.response_time = response_time.to_msg()
        goal_handle.succeed()

        print("Server has called succeed()")    
        return result

def main(args=None):
    rclpy.init(args=args)
    server_node = ServerNode()

    try:
        rclpy.spin(server_node)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        server_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()