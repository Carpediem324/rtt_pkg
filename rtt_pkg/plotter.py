#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import glob
import getpass
import os, time

user = getpass.getuser()
if user == 'root': #도커에서 사용하기 위한 예외코드입니다..
    parent_dir = '/home/shh/'
else:
    parent_dir = os.path.join('/home', user)

TARGET_SAVE_PATH = os.path.join(parent_dir, 'ros2_ws', 'src', 'rtt_pkg', 'rttdata')

class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')
        self.plot_rtt_data()

    def plot_rtt_data(self):
        # Get a list of all files in the rttdata directory
        files = glob.glob(TARGET_SAVE_PATH+'/*.txt')

        if not files:
            print('No RTT data files found.')
            return

        # Find the most recent file
        most_recent_file = max(files, key=os.path.getctime)

        # Read RTT data from the most recent file
        with open(most_recent_file, 'r') as file:
            rtt_data = [float(line.strip()) for line in file]

        # Calculate the average RTT
        average_rtt = sum(rtt_data) / len(rtt_data)

        # Plot the RTT data
        plt.plot(rtt_data, marker='o', linestyle='-', label='RTT')
        plt.axhline(y=average_rtt, color='r', linestyle='--', label='Average RTT')
        
        # Add text for the average RTT
        plt.text(0.5, average_rtt, 'Average RTT: {:.3f}'.format(average_rtt), color='r')
        
        plt.title('Round Trip Time (RTT) Over Time')
        plt.xlabel('Time (seconds)')
        plt.ylabel('RTT (seconds)')
        plt.legend()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
