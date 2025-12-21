#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import time
import sys


class CompleteMazeTester(Node):
    def __init__(self):
        super().__init__('complete_maze_tester')
        
        # Initialize navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Initialize map loading service client
        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        
        # Wait for required servers and services
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Nav2 action server...')
            
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for map_server service...')
            
        self.get_logger().info('✅ All services ready')

    def send_goal(self, x, y, theta=0.0, frame_id='map'):
        """Send a navigation goal to Nav2."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        q = quaternion_from_euler(0, 0, theta)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        
        self.get_logger().info(f'📍 Sending goal: ({x:.2f}, {y:.2f})')
        
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is None:
            self.get_logger().error('❌ Failed to send goal')
            return False
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected')
            return False
            
        self.get_logger().info('✅ Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        
        if result_future.result() is None:
            self.get_logger().error('❌ Navigation timeout or failure')
            return False
            
        status = result_future.result().status
        if status == 3:  # GoalStatus.SUCCEEDED
            self.get_logger().info('🎉 Navigation succeeded!')
            return True
        else:
            self.get_logger().error(f'❌ Navigation failed with status: {status}')
            return False

    def switch_to_phase2_map(self):
        """Switch map server to Phase 2 map (original maze with obstacles)."""
        self.get_logger().info('🔄 Switching to Phase 2 map...')
        
        req = LoadMap.Request()
        req.map_url = 'package://simple_maze_bot/maps/maze_original.yaml'
        
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            response = future.result()
            if response.result == LoadMap.Response.RESULT_SUCCESS:
                self.get_logger().info('✅ Map switch successful')
                time.sleep(3.0)  # Wait for AMCL to stabilize
                return True
            else:
                self.get_logger().error(f'❌ Map switch failed: {response.result}')
                return False
        else:
            self.get_logger().error('❌ Map switch service call failed')
            return False

    def run_complete_test(self):
        """Execute the two-phase navigation test sequence."""
        self.get_logger().info('🚀 Starting complete maze navigation test...')
        
        # Phase 1: Navigate to START point using soft-obstacle map
        self.get_logger().info('🔷 Phase 1: Navigating to START (0.22, 1.65)')
        if not self.send_goal(0.22, 1.65, theta=0.0):
            self.get_logger().error('❌ Phase 1 failed')
            return False
        
        time.sleep(2.0)
        
        # Change map to enforce hard obstacles
        if not self.switch_to_phase2_map():
            return False
        
        # Phase 2: Navigate to END point avoiding obstacles
        self.get_logger().info('🔶 Phase 2: Navigating to END (1.58, 1.65)')
        if not self.send_goal(1.58, 1.65, theta=0.0):
            self.get_logger().error('❌ Phase 2 failed')
            return False
        
        self.get_logger().info('🏆 Complete maze task SUCCESS!')
        return True


def main(args=None):
    rclpy.init(args=args)
    tester = CompleteMazeTester()
    
    try:
        success = tester.run_complete_test()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        tester.get_logger().info('Interrupted by user')
    except Exception as e:
        tester.get_logger().error(f'Test exception: {str(e)}')
        sys.exit(1)
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
