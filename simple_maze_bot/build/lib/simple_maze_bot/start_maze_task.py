# scripts/start_maze_task.py

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import sys

def navigate_to(x, y, node, client):
    """辅助函数：导航到指定 (x, y)"""
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation.w = 1.0

    node.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f})')
    future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)

    if not future.result().accepted:
        return False

    result_future = future.result().get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    return result_future.result().status == 4

def main():
    rclpy.init()
    node = rclpy.create_node('maze_task_executor')

    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('Nav2 not ready!')
        rclpy.shutdown()
        sys.exit(1)

    # ==============================
    # 🎯 正式任务：修改这里的坐标！
    # ==============================
    waypoints = [
        (0.77, 1.6),   # 示例：终点
        # 可扩展为多点：
        # (1.0, 1.5),
        # (2.0, 1.0),
        # (2.5, 2.5)
    ]

    success = True
    for x, y in waypoints:
        if not navigate_to(x, y, node, client):
            node.get_logger().error(f'Failed at waypoint ({x}, {y})')
            success = False
            break

    if success:
        node.get_logger().info('🎉 Maze task completed successfully!')
        sys.exit(0)
    else:
        node.get_logger().error('💥 Maze task failed.')
        sys.exit(1)

if __name__ == '__main__':
    main()