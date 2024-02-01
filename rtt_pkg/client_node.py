#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import ROSClock
from rclpy.executors import SingleThreadedExecutor

from custom_interfaces2.action import Rtt

import datetime
import getpass
import os, time
import shutil

user = getpass.getuser()
if user == 'root':
    parent_dir = '/home/shh/'
else:
    parent_dir = os.path.join('/home', user)

TARGET_SAVE_PATH = os.path.join(parent_dir, 'ros2_ws', 'src', 'rtt_pkg', 'rttdata')

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.client = ActionClient(self, Rtt, 'my_action')
        self.client.wait_for_server()
        filename = TARGET_SAVE_PATH+'/rtt_data.txt'
        if os.path.exists(filename):
            backup_dir_base_name = 'Backup'
            i = 0
            while True:
                i += 1
                backup_dir_name = f'{TARGET_SAVE_PATH}/{backup_dir_base_name}{i}'
                if not os.path.exists(backup_dir_name):
                    os.makedirs(backup_dir_name)
                    shutil.move(filename, os.path.join(backup_dir_name, 'rtt_data.txt'))
                    break

    def send_request(self):
        for _ in range(60):  # Repeat 60 times
            request_time = ROSClock().now()
            self.get_logger().info('Sending request time: {}'.format(request_time.to_msg()))

            goal = Rtt.Goal()
            goal.request_time = request_time.to_msg()
            send_goal_future = self.client.send_goal_async(goal)

            # Wait until `send_goal_future` is done
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()
            if goal_handle.accepted:
                get_result_future = goal_handle.get_result_async()

                # Wait until `get_result_future` is done
                rclpy.spin_until_future_complete(self, get_result_future)

                if get_result_future.done():
                    result = get_result_future.result()
                    result_time = ROSClock().now()
                    #서버에서받은시간
                    self.get_logger().info('Server Received response time: {}'.format(result.result.response_time))
                    #내가 받은시간
                    self.get_logger().info('Client Received result time: {}'.format(result_time.to_msg()))

                    rtt = result_time - request_time
                    self.get_logger().info('Round Trip Time (RTT): {}'.format(rtt.nanoseconds / 1e9))

                    self.save_rtt_data_to_file(rtt.nanoseconds / 1e9)
                else:
                    self.get_logger().info('Failed to get a result')
            else:
                self.get_logger().info('Goal was rejected by the server')

            time.sleep(1)


    def save_rtt_data_to_file(self, rtt_value):
        filename = TARGET_SAVE_PATH+'/rtt_data.txt'
        # Save RTT data to the text file
        with open(filename, 'a') as file:
            file.write(str(rtt_value) + '\n')

def main(args=None):
    rclpy.init(args=args)
    client_node = ClientNode()
    client_node.send_request()
    rclpy.shutdown()

if __name__ == '__main__':
    main()