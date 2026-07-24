#!/usr/bin/env python3

import rclpy
from rclpy.qos import (
        QoSProfile,
        ReliabilityPolicy,
        DurabilityPolicy,
        HistoryPolicy
)

from rclpy.node import Node

from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np


class LineMapNode(Node):
    def __init__(self):
        super().__init__('line_map_node')

        self.resolution = 0.06
        self.width = 600
        self.height = 600
        self.origin_x = 0
        self.origin_y = 0

        self.path_radius = 7 
        self.wall_radius = 1

        self.map_data = np.full((self.height, self.width), -1, dtype=np.int8)

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        map_qos = QoSProfile(
           reliability=ReliabilityPolicy.RELIABLE,
           durability=DurabilityPolicy.TRANSIENT_LOCAL,
           history=HistoryPolicy.KEEP_LAST,
           depth=1
	)
        

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )

        self.timer = self.create_timer(0.5, self.publish_map)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)

        if not (0 <= mx < self.width and 0 <= my < self.height):
            return

        for dy in range(-self.wall_radius, self.wall_radius + 1):
            for dx in range(-self.wall_radius, self.wall_radius + 1):
                nx = mx + dx
                ny = my + dy

                if not (0 <= nx < self.width and 0 <= ny < self.height):
                    continue

                dist = (dx * dx + dy * dy) ** 0.5

                if dist <= self.path_radius:
                    self.map_data[ny, nx] = 0
                elif dist <= self.wall_radius:
                    if self.map_data[ny, nx] == -1:
                        self.map_data[ny, nx] = 100

    def publish_map(self):
        grid = OccupancyGrid()

        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'

        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height

        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0

        grid.data = self.map_data.flatten().tolist()

        self.map_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = LineMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

