#!/usr/bin/env python3
"""ROS 2 Humble: Task A -> Task B -> Task C in one process.

Examples:
  python3 main.py --marker-id 5           # A+B+C, automatic docking
  python3 main.py --marker-id 5 --dry-c   # A+B+C, C measurement only
  python3 main.py --marker-id 5 --skip-ab # C docking from current location
"""
import argparse
import cv2
import rclpy
import time
from .Task_A import TaskA
from .Task_B import TaskB
from .Task_C import Dock


def stop_and_destroy(node, is_a=False):
    if node is None:
        return
    try:
        node.stop_robot() if hasattr(node, 'stop_robot') else node.cmd()
    except Exception as exc:
        print(f'WARN stop failed: {exc}', flush=True)
    if is_a:
        try:
            node.close_camera()
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        except cv2.error:
            pass
    node.destroy_node()


def run_ab(node, name):
    try:
        node.get_logger().info(f'Task {name} started')
        while rclpy.ok() and node.state not in ('COMPLETE', 'ERROR'):
            rclpy.spin_once(node, timeout_sec=.1)
        return node.state == 'COMPLETE'
    finally:
        stop_and_destroy(node, is_a=(name == 'A'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--marker-id', type=int, required=True, choices=range(1,6))
    parser.add_argument('--skip-ab', action='store_true', help='Start at Task C from current pose')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--move-c', dest='move_c', action='store_true',
                      help='Enable Task C docking motion (default)')
    mode.add_argument('--dry-c', dest='move_c', action='store_false',
                      help='Measure Task C goal only; do not move')
    parser.set_defaults(move_c=True)
    parser.add_argument('--scan-topic', default='/scan', help='LiDAR topic for Task C')
    parser.add_argument('--clearance-tolerance-cm', type=float, default=0.5,
                        help='Allowed error around the 10 cm docking target (default: 0.05 cm)')
    args = parser.parse_args()
    rclpy.init()
    try:
        if not args.skip_ab:
            if not run_ab(TaskA(), 'A'):
                print('Task A failed; B/C not started', flush=True)
                return 1
            print('Task A complete; OpenCV camera window closed.', flush=True)
            time.sleep(2.0)
            if not run_ab(TaskB(args.marker_id), 'B'):
                print('Task B failed; C not started', flush=True)
                return 1
            print(f'Task B finished; forwarding ID={args.marker_id} to C', flush=True)
        task_c = Dock(
            args.marker_id,
            move=args.move_c,
            target_clearance=0.10,
            clearance_tolerance=max(0.002, args.clearance_tolerance_cm / 100.0),
            scan_topic=args.scan_topic,
        )
        try:
            while rclpy.ok() and task_c.state not in ('READY', 'COMPLETE', 'ERROR'):
                rclpy.spin_once(task_c, timeout_sec=.1)
            print(f'Task C finished with state={task_c.state}', flush=True)
            return 0 if task_c.state in ('READY', 'COMPLETE') else 1
        finally:
            stop_and_destroy(task_c)
    except KeyboardInterrupt:
        print('Interrupted; stopping robot', flush=True)
        return 130
    finally:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
