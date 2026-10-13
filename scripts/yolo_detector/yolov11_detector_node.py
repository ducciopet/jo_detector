#!/usr/bin/env python3

import os
import sys
import traceback

import rclpy
from rclpy.logging import get_logger
from ament_index_python.packages import get_package_share_directory


def main():
    _pkg_share = get_package_share_directory("jo_detector")
    _scripts_dir = os.path.join(_pkg_share, "scripts", "yolo_detector")
    if _scripts_dir not in sys.path:
        sys.path.insert(0, _scripts_dir)

    try:
        from yolov11_detector import yolo_detector
    except Exception as e:
        get_logger("yolov11_detector_node").fatal(
            f"Import failed — check torch/ultralytics installation:\n{traceback.format_exc()}"
        )
        raise SystemExit(1)

    rclpy.init()
    try:
        node = yolo_detector()
    except Exception as e:
        get_logger("yolov11_detector_node").fatal(
            f"Node initialization failed:\n{traceback.format_exc()}"
        )
        rclpy.shutdown()
        raise SystemExit(1)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
