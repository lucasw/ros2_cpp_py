#!/usr/bin/env python3

# Jacob Beck
# Node to preview and save images on demand

import rclpy
from rclpy.node import Node

from ros2_cpp_py.msg import Test
from ros2_cpp_py.srv import Test2

class PyTalker(Node):
    def __init__(self):
        super().__init__('pytalker')
        self._count = 0
        self._log_timer = self.create_timer(1, self._log_callback)

        return

    def _log_callback(self):
        self.get_logger().info("Hello! {}".format(self._count))
        self._count += 1

        return


def main(args=None):
    rclpy.init(args=args)

    print(Test.FOO)
    print(Test2.Request.SOME)

    pytalker = PyTalker()
    rclpy.spin(pytalker)

    pytalker.destroy_node()
    rclpy.shutdown()

    return


if __name__ == '__main__':
    main()
