#!/usr/bin/env python3
# scripts/test_complete_simulation.py
"""
完整两阶段导航自动化测试脚本：

1. Phase 1: 从随机位置导航到 START 点 (0.22, 1.65)
   - 使用 maze_no_soft.pgm（软障碍可通行）

2. Phase 2: 切换地图为 maze_original.pgm（软障碍不可通行）
   - 从 START 点导航到终点 (1.58, 1.65)，必须避开软障碍

依赖：
- Nav2 的 /change_map 服务（由 map_server 提供）
- AMCL 定位
- NavigateToPose 动作接口
"""

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
        
        # 创建导航动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 创建地图切换服务客户端
        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        
        # 等待服务可用
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('等待导航服务器...')
            
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待地图切换服务...')
            
        self.get_logger().info('✅ 所有服务已就绪')

    def send_goal(self, x, y, theta=0.0, frame_id='map'):
        """发送导航目标"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        q = quaternion_from_euler(0, 0, theta)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        
        self.get_logger().info(f'📍 发送目标: ({x:.2f}, {y:.2f}) @ {theta:.2f} rad')
        
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is None:
            self.get_logger().error('❌ 导航目标发送失败')
            return False
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 导航目标被拒绝')
            return False
            
        self.get_logger().info('✅ 导航目标已接受，等待结果...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        
        if result_future.result() is None:
            self.get_logger().error('❌ 导航超时或失败')
            return False
            
        status = result_future.result().status
        if status == 3:  # GoalStatus.SUCCEEDED
            self.get_logger().info('🎉 导航成功完成！')
            return True
        else:
            self.get_logger().error(f'❌ 导航失败，状态码: {status}')
            return False

    def switch_to_phase2_map(self):
        """切换到 Phase 2 地图（包含软障碍）"""
        self.get_logger().info('🔄 切换到 Phase 2 地图（maze_original.yaml）...')
        
        req = LoadMap.Request()
        req.map_url = 'package://simple_maze_bot/maps/maze_original.yaml'
        
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            response = future.result()
            if response.result == LoadMap.Response.RESULT_SUCCESS:
                self.get_logger().info('✅ 地图切换成功！')
                # 等待 AMCL 重置并重新定位
                time.sleep(3.0)
                return True
            else:
                self.get_logger().error(f'❌ 地图切换失败，错误码: {response.result}')
                return False
        else:
            self.get_logger().error('❌ 地图切换服务调用失败')
            return False

    def run_complete_test(self):
        """执行完整两阶段测试"""
        self.get_logger().info('🚀 开始完整迷宫导航测试...')
        
        # === Phase 1: 导航到 START 点 ===
        self.get_logger().info('🔷 Phase 1: 导航到 START 点 (0.22, 1.65)')
        success1 = self.send_goal(0.22, 1.65, theta=0.0)
        
        if not success1:
            self.get_logger().error('❌ Phase 1 失败，终止测试')
            return False
        
        time.sleep(2.0)  # 短暂等待
        
        # === 切换地图 ===
        self.get_logger().info('🔄 准备进入 Phase 2...')
        if not self.switch_to_phase2_map():
            self.get_logger().error('❌ 地图切换失败，终止测试')
            return False
        
        # === Phase 2: 导航到终点（避开软障碍）===
        self.get_logger().info('🔶 Phase 2: 导航到终点 (1.58, 1.65)，避开软障碍')
        success2 = self.send_goal(1.58, 1.65, theta=0.0)
        
        if not success2:
            self.get_logger().error('❌ Phase 2 失败')
            return False
        
        self.get_logger().info('🏆 完整迷宫任务 SUCCESS！')
        return True


def main(args=None):
    rclpy.init(args=args)
    
    tester = CompleteMazeTester()
    
    try:
        success = tester.run_complete_test()
        if success:
            print("\n✅ 完整测试通过！")
            sys.exit(0)
        else:
            print("\n❌ 完整测试失败！")
            sys.exit(1)
            
    except KeyboardInterrupt:
        tester.get_logger().info('测试被用户中断')
    except Exception as e:
        tester.get_logger().error(f'测试发生异常: {str(e)}')
        sys.exit(1)
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()