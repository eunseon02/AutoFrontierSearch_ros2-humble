# I will write a Python program that demonstrates how to analyze map data to detect frontiers and set a goal towards a detected frontier.
# This program is a basic implementation and should be adapted for specific requirements and the actual robot hardware.

# Import necessary ROS2 libraries
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatusArray
import numpy as np
import math
import random
from rclpy.time import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy

class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node')
        durable_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile=durable_qos)
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.map_data = None
        self.map_array = None
        self.map_metadata = None
        self.goal_status_subscriber = self.create_subscription(
            GoalStatusArray,
            '/follow_path/_action/status',  # Replace with the actual topic name
            self.goal_status_callback,
            10)
        self.goal_reached = True
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/chassis/odom',
            self.odom_callback,
            10)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.map_info = None
        self.timer = self.create_timer(5, self.timer_callback)
        self.last_goal  = None
        self.last_heading = None
        self.heading_threshold = 0.5
        self.republish_delay = 8.0

    def goal_status_callback(self, msg):
        if msg.status_list:
            # 最新のゴールステータスを取得
            current_status = msg.status_list[-1].status
            print(current_status)
            # ゴールに達していない場合、Goal_reached を False に設定
            if current_status == 3:
                self.goal_reached = False
                self.get_logger().info("Goal SUCCEEDED → ready for next frontier")
            elif current_status == 2:
                self.goal_reached = True
                print("active.")
            elif current_status == 4:
                self.goal_reached = False
                print("goal abort.")
            else:
                self.goal_reached = False
                self.get_logger().info("Goal state changed → ready for new frontier")

            # 現在のゴールの状態をログに記録
            self.get_logger().info(f"Goal reached: {self.goal_reached}")
    
    def map_callback(self, msg):
        self.get_logger().info(">>> map_callback 호출됨")

        self.map_data = msg.data
        self.map_array = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_metadata = msg.info
        self.map_info = msg.info
        
        
                
    def timer_callback(self):
        now = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec*1e-9
        if now - self.last_publish_time < self.republish_delay:
            return

        if self.map_array is None:
            self.get_logger().debug("map_array 아직 없음, 기다리는 중") 
            return
        self.get_logger().info(f"timer fired: map_array.any()={self.map_array.any()}, goal_reached={self.goal_reached}")
        frontiers = self.detect_frontiers(map_info=self.map_info)
        if frontiers is None:
            print("Goal not reached, waiting.")
            # 아직 목표를 보내지 않았거나, 이전 목표에 도달 대기 중
            return

        goal = self.select_goal(frontiers, self.map_array)
        if goal is not None:
            self.publish_goal(goal)
            self.goal_reached = False
            self.get_logger().info(f"Published new goal: {goal}")
                
    

    
    def odom_callback(self, msg):
        # Update the robot's current position based on the odometry data
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
            
    def is_valid_cell(self, x, y, map_info):
        # セルがマップの有効範囲内にあるか確認
        # 例えば、マップの境界をチェック
        return 0 <= x < map_info.width and 0 <= y < map_info.height
    

    def is_near_wall(self, x, y, map_data, threshold):
        # この関数は、指定された座標が壁に近いかどうかを判定します。
        # x, yはフロンティアの座標、map_dataはマップのデータ、
        # thresholdは壁と判定するための距離のしきい値です。

        # マップの周囲のセルを調べる
        for i in range(-threshold, threshold+1):
            for j in range(-threshold, threshold+1):
                # マップデータ内での位置を計算
                check_x = x + i
                check_y = y + j

                # マップの範囲内かどうか確認
                if 0 <= check_x < map_data.shape[0] and 0 <= check_y < map_data.shape[1]:
                    if map_data[check_x, check_y] == 100:  # 壁を示す値
                        return True
        return False
    
    def detect_frontiers(self, map_info ):
        # Detecting frontiers (edges of explored and unexplored areas)
        # For simplicity, this is a very basic implementation and should be enhanced for real-world applications.
        if not self.goal_reached:
            return None 
        frontiers = []
            
        for y in range(map_info.height):
            for x in range(map_info.width):
                # マップデータの範囲内にあるセルのみを検討
            
                
                    if self.map_array[y, x] == -1:  # -1 indicates unknown area in the map
                        neighbors = self.map_array[y-1:y+2, x-1:x+2]
                        if 0 in neighbors:  # 0 indicates free space in the map

                            frontiers.append((x, y))
        self.get_logger().info(
            f"Detected {len(frontiers)} frontiers "
            f"(map: {map_info.width}×{map_info.height}, res={map_info.resolution})"
        )
        return frontiers

        
    def select_goal(self, frontiers):
        rx, ry = self.robot_x, self.robot_y
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y

        # 1) 그리드 인덱스 → 월드 좌표 리스트로 변환
        world_pts = [((ix*res + ox), (iy*res + oy)) for ix, iy in frontiers]

        # 2) 거리 기준 오름차순 정렬
        world_pts.sort(key=lambda p: math.hypot(p[0]-rx, p[1]-ry))

        # 3) 이전 목표 방향과의 일관성 검사
        if self.last_heading is not None:
            for wx, wy in world_pts:
                ang = math.atan2(wy-ry, wx-rx)
                diff = abs((ang - self.last_heading + math.pi) % (2*math.pi) - math.pi)
                if diff < self.heading_threshold:
                    self.last_goal = (wx, wy)
                    self.last_heading = ang
                    return wx, wy

        # 4) 후보 없으면 가장 가까운 목표
        if world_pts:
            wx, wy = world_pts[0]
            ang = math.atan2(wy-ry, wx-rx)
            self.last_goal = (wx, wy)
            self.last_heading = ang
            return wx, wy

        return None


    def publish_goal(self, goal):
        real_x, real_y = goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = real_x
        goal_msg.pose.position.y = real_y
        goal_msg.pose.position.z = 0.0

        dx = real_x - self.robot_x
        dy = real_y - self.robot_y
        yaw = math.atan2(dy, dx)

        # z-축 회전만 있는 쿼터니언 생성
        half_yaw = yaw * 0.5
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw

        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal → x: {real_x:.2f}, y: {real_y:.2f}, yaw: {yaw:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
