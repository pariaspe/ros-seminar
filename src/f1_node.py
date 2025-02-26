#! /usr/bin/env python3

"""
f1_node.py
"""

import rclpy
from rclpy.node import Node


class F1Node(Node):
    """F1 Node
    """

    def __init__(self) -> None:
        super().__init__('f1_node')

        # Publishers, subscribers...


def main(args=None):
    """Main method
    """
    rclpy.init(args=args)

    node = F1Node()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
