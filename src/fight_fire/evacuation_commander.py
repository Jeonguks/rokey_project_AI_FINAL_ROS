import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import TaskResult

import json
import time
import math

class State:
    IDLE = 0
    UNDOCKING = 1
    MOVINGA = 2
    SEARCHING = 3
    GUIDING = 4
    MOVINGAIN = 5
    FIND=6

class EvacuationCommander(Node):
    def __init__(self):
        super().__init__('evacuation_commander')
        
        self.namespace = 'robot2'

        # --- [TurtleBot4 Navigator 설정] ---
        self.nav = TurtleBot4Navigator(namespace=self.namespace)
        
        # 통신 설정
        self.create_subscription(String, 'detected_objects', self.detection_callback, 10)
        
        # --- [좌표 설정] ---
        # 탐색 웨이포인트 [x, y, yaw]
        self.search_waypoints = [
            [3.9223, -0.3839, 0.0],
            [3.3106, -1.7768, 0.0],
            [3.1855, -3.7011, 0.0],
        ]
        # 대피소 [x, y, yaw]
        self.evac_point = [0.972021, 0.383458, 1.0]
        
        # --- [파라미터] ---
        self.valid_data_duration = 1.0
        
        # --- [상태 변수] ---
        self.state = State.IDLE
        self.last_seen_time = 0.0
        
        # 네비게이션 활성화 대기 (필수)
        # self.get_logger().info("⏳ Waiting for Nav2...")
        # self.nav.waitUntilNav2Active() 
        # (주석: TB4Navigator는 내부적으로 처리하지만, 필요시 주석 해제)

        # 설명: 'cmd_vel'이라는 토픽으로 Twist 메시지(속도 명령)를 보내는 발신자(Publisher)를 생성합니다.
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info("✅ TB4 Commander Ready. Monitoring 'A' value...")

        self.is_moving = False

    def detection_callback(self, msg):
        """
        JSON 예시: {"A": "stand", "B": "fire"}
        """
        if self.is_moving:
            return
        
        try:
            data = json.loads(msg.data)
            if data.get("A") == "stand":
                self.get_logger().info("🚀 Command Received! Calling go_to_A...")
                self.is_moving = True # 이동 시작 플래그
                self.run_mission()
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
#==============================================================================

    

    def run_mission(self):
        # 2. 탐색
        self.get_logger().info("Starting Mission.")
        self.state = State.SEARCHING
        target_found = self.go_to_A()
        
        if target_found:
            self.state = State.SEARCHING
            rotate_search = self.rotate_90_degrees()
        else:
            self.get_logger().info(f"A방 도착 실패{target_found}")

        if rotate_search:
            self.state = State.MOVINGAIN
            self.go_to_A_in()
        else:
            self.get_logger().info(f"A방 안쪽 도착 실패")

        if self.state == State.MOVINGAIN:
            self.rotate_90_degrees()
            self.state = State.FIND
        else:
            self.get_logger().info(f"탐색 실패")

        if self.state == State.FIND:
            self.guide_sequence()


        

        # if target_found:
        #     # 3. 구조
        #     self.get_logger().info("🎉 Target confirmed! Switching to Guide Mode.")
        #     self.state = State.GUIDING
        #     # self.guide_sequence()
        # else:
        #     # --- [고칠 부분] 발견 실패 시 복귀 로직 ---
        #     self.get_logger().warn("❌ Mission Failed: Could not find target.")
        #     self.get_logger().info("🏠 Returning to Base (Dock)...")
        #     self.return_to_base() # 복귀 함수 호출
        #     self.state = State.IDLE

    
    def return_to_base(self):
        """
        탐색 실패 시 집(Dock)으로 복귀하는 함수
        """
        goal_pose = self.nav.getPoseStamped([3.7, 2.1], TurtleBot4Directions.SOUTH_EAST)
        self.nav.startToPose(goal_pose)
        # 방법 1: TurtleBot4 자동 도킹 기능 사용
        # (네비게이션 맵 상에 도킹 스테이션 위치가 설정되어 있어야 함)
        self.nav.dock()

        # 도킹 완료될 때까지 대기
        while not self.nav.isTaskComplete():
            # 복귀 중에도 혹시 'stand'가 발견되면 멈추고 싶다면 여기에 로직 추가 가능
            # if self.is_stand_fresh(): ...
            pass
            
        self.get_logger().info("🏠 Arrived at Dock. Charging...")
        
        # 방법 2: 만약 도킹이 아니라 특정 좌표(예: [0,0,0])로 가고 싶다면 아래 주석 해제 사용
        # home_pose = self.create_pose([0.0, 0.0, 0.0])
        # self.nav.startToPose(home_pose)
        # while not self.nav.isTaskComplete(): pass

    def go_to_A(self):
        # Start on dock
        if self.nav.getDockedStatus():
            self.get_logger().info("🔌 Undocking first...")
            self.nav.undock()

        # Wait for Nav2
        self.nav.waitUntilNav2Active()

        # Set goal poses
        goal_pose = []
        goal_pose.append(self.nav.getPoseStamped([3.9223, -0.3839], TurtleBot4Directions.SOUTH_EAST))
        goal_pose.append(self.nav.getPoseStamped([3.3106, -1.7768], TurtleBot4Directions.SOUTH_EAST))

        # Navigate through poses
        self.nav.startThroughPoses(goal_pose)

        # 2. 도착할 때까지 기다리는 루프 (가장 중요!)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            # 필요하다면 여기서 남은 거리 등을 로그로 찍을 수 있습니다.
            # self.get_logger().info(f'이동 중... 남은 거리: {feedback.distance_remaining}')
            
            # 0.1초 정도 대기하며 루프 반복 (CPU 과부하 방지)
            # rclpy.spin_once() 같은 처리가 필요할 수도 있음 (구조에 따라 다름)
            pass

        # 3. 루프가 끝나면(도착하거나 취소되면) 최종 결과 확인
        result = self.nav.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("✅ A방 진입 완료 (성공)")
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().info("⚠️ 이동 취소됨")
            return False
        elif result == TaskResult.FAILED:
            self.get_logger().info("❌ 이동 실패 (경로 막힘 등)")
            return False
        else:
            return False


    def go_to_A_in(self):
        goal_pose = self.nav.getPoseStamped([3.1855, -3.7011], TurtleBot4Directions.SOUTH_EAST)
        self.nav.startToPose(goal_pose)

    # [1] 90도 회전 함수
    def rotate_90_degrees(self):
        self.get_logger().info("탐색을 위해 90도 회전합니다...")

        # [핵심] TurtleBot4Navigator에는 spin 기능이 이미 있습니다!
        # spin_dist: 회전할 각도 (라디안). 1.57 = 90도
        # time_allowance: 10초 안에 못 돌면 실패 처리
        self.nav.spin(spin_dist=-2.07, time_allowance=10)

        # [중요] 회전이 끝날 때까지 기다리는 루프
        while not self.nav.isTaskComplete():
            # 피드백을 받아와서 로그를 찍어도 됩니다 (선택사항)
            # feedback = self.navigator.getFeedback()
            pass  # 다 돌 때까지 대기

        # 결과 확인
        result = self.nav.getResult()
        
        # TaskResult.SUCCEEDED와 비교해야 하지만, 간단히 성공 로그 출력
        self.get_logger().info("회전 탐색 완료!")
        return True

#=====================================================================================

    def guide_sequence(self):
        self.get_logger().info("Step 3: Guiding to Evacuation Point...")
        # evac_pose = self.create_pose(self.evac_point)
        # self.nav.startToPose(evac_pose)
        # goal_pose = self.nav.getPoseStamped([0.972021, 0.383458], TurtleBot4Directions.NORTH)
        # self.nav.startToPose(goal_pose)
        self.nav.waitUntilNav2Active()

        # Set goal poses
        goal_pose = []
        goal_pose.append(self.nav.getPoseStamped([3.92, -1.09], TurtleBot4Directions.SOUTH_EAST))
        goal_pose.append(self.nav.getPoseStamped([0.972021, 0.383458], TurtleBot4Directions.SOUTH_EAST))

        # Navigate through poses
        self.nav.startThroughPoses(goal_pose)

        # 2. 도착할 때까지 기다리는 루프 (가장 중요!)
        while not self.nav.isTaskComplete():
            # feedback = self.nav.getFeedback()
            pass

        # 3. 루프가 끝나면(도착하거나 취소되면) 최종 결과 확인
        result = self.nav.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("✅ 대피로 도착 (성공)")
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().info("⚠️ 이동 취소됨")
            return False
        elif result == TaskResult.FAILED:
            self.get_logger().info("❌ 이동 실패 (경로 막힘 등)")
            return False
        else:
            return False
        
        last_check_time = time.time()
        
        # while not self.nav.isTaskComplete():
        #     # 3초마다 확인
        #     if time.time() - last_check_time > 3.0:
        #         self.nav.cancelTask() # 잠시 멈춤
                
        #         self.check_follower()
        #         # if not self.check_follower():
        #         #     if not self.handle_lost_follower():
        #         #         self.get_logger().error("사람을 완전히 놓쳤습니다. 미션 종료.")
        #         #         return 
                
        #         self.get_logger().info("Resuming guide...")
        #         self.nav.startToPose(evac_pose)
        #         last_check_time = time.time()
        
        self.get_logger().info("✅ Mission Complete.")
        self.state = State.IDLE

    def create_pose(self, coord):
        pose = PoseStamped()    # 위치(Pose)와 도장(Stamp, 시간+기준좌표)이 찍힌 빈 편지봉투를 만듭니다.
        pose.header.frame_id = 'map' # 이 좌표가 '지도(Map)' 기준이라는 뜻입니다. (로봇 기준인 base_link가 아님). 즉, 지도 상의 절대 좌표 (x, y)로 이동하라는 의미
        pose.header.stamp = self.get_clock().now().to_msg() # 현재 시간을 찍습니다. Nav2는 시간이 너무 오래 지난 명령은 "낡은 데이터"로 취급해 무시할 수 있으므로, 항상 **현재 시간(now())**을 넣어줘야 합니다.
        pose.pose.position.x = float(coord[0])  # 리스트의 0번째, 1번째 값을 꺼내 x, y 좌표에 넣습니다.
        pose.pose.position.y = float(coord[1])
        yaw = coord[2]
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose


    def is_stand_fresh(self):
        """최근 1초 내에 'stand' 감지 여부"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        return True #(current_time - self.last_seen_time) < self.valid_data_duration

    def check_follower(self):
        """뒤돌아보기 (Visual check)"""
        self.get_logger().info("👀 Checking behind...")
        
        # [1] 180도(3.14 라디안) 회전
        # 터틀봇4 네비게이터 사용 시
        self.nav.spin(spin_dist=3.14, time_allowance=10)
        while not self.nav.isTaskComplete():
            pass
        # [2] 회전이 끝난 후 잠시 대기 (카메라 초점/인식 안정화)
        time.sleep(1.0)
        
        is_following = False
        if self.is_stand_fresh():
            self.get_logger().info("✅ Follower confirmed (Visual)")
            is_following = True
        else:
            self.get_logger().warn("⚠️ No target visible!")
        
        self.nav.spin(spin_dist=3.14, time_allowance=10) # 혹은 -3.14
        while not self.nav.isTaskComplete():
            pass

        return is_following

    def handle_lost_follower(self):
        self.get_logger().warn("🛑 Follower LOST! Waiting 5s...")
        time.sleep(5.0) # 5초간 그 자리에 얼음(정지)
        if self.check_follower():
            return True
        else:
            self.get_logger().error("💀 ABORT MISSION.")
            # TB4Navigator는 lifecycleShutdown이 없을 수 있음.
            self.state = State.IDLE 
            return False

    
        


def main(args=None):
    rclpy.init(args=args)
    node = EvacuationCommander()
    rclpy.spin(node)
    rclpy.shutdown()