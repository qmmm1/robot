# scripts/go_to_start.py

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import sys

def main():
    rclpy.init()
    node = rclpy.create_node('go_to_start')

    # 创建 Action 客户端
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    node.get_logger().info('Waiting for Nav2 action server...')
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('Nav2 action server not available!')
        rclpy.shutdown()
        sys.exit(1)

    # 构建目标位姿（START 点）
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = 1.58  # 米
    goal_msg.pose.pose.position.y = 0.15
    goal_msg.pose.pose.orientation.w = 1.0  # 朝向 0 弧度

    node.get_logger().info('Sending goal to START point (0.1, 0.1)...')
    future = client.send_goal_async(goal_msg)

    # 等待结果
    rclpy.spin_until_future_complete(node, future)
    if not future.result().accepted:
        node.get_logger().error('Goal rejected by Nav2!')
        rclpy.shutdown()
        sys.exit(1)

    result_future = future.result().get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    status = result_future.result().status
    if status == 4:  # GoalStatus.SUCCEEDED
        node.get_logger().info('✅ Successfully reached START point!')
        rclpy.shutdown()
        sys.exit(0)
    else:
        node.get_logger().error(f'❌ Failed to reach START. Status: {status}')
        rclpy.shutdown()
        sys.exit(1)

if __name__ == '__main__':
    main()