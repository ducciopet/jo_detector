#!/usr/bin/env python3

import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory

# Ensure the packaged scripts directory is on PYTHONPATH for imports.
_pkg_share = get_package_share_directory("jo_detector")
_scripts_dir = os.path.join(_pkg_share, "scripts", "yolo_detector")
if _scripts_dir not in sys.path:
	sys.path.insert(0, _scripts_dir)

from yolo_detector import yolo_detector


def main():
	rclpy.init()
	node = yolo_detector()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()

