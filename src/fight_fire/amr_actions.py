#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Twist, Point
from std_msgs.msg import Bool,String
from irobot_create_msgs.msg import AudioNoteVector, AudioNote

from nav2_simple_commander.robot_navigator import TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import math
import json


class State:
    IDLE = 0
    UNDOCKING = 1
    MOVINGA = 2
    SEARCHING = 3
    GUIDING = 4
    MOVINGAIN = 5
    FIND=6


class RobotActionLib:
    """
    FullSequenceTest가 트리거를 담당하고,
    RobotActionLib는 '행동(액션)'만 제공하는 라이브러리로 사용.
    """

    def __init__(self, node: Node):
        self.node = node
        self.namespace = self.node.get_namespace() 
        self.other_namespace = "/robot6"

        if self.namespace == "/robot2":
            self.other_namespace = "/robot6"
        else:
            self.other_namespace = "/robot2"


        self.help_signal_pub = self.node.create_publisher(
            Bool, f'{self.namespace}/signal/help', 10
        )
        self.help_coordinate_pub = self.node.create_publisher(
            Point, f'{self.namespace}/signal/coordinate', 10
        )

        # 상대 로봇 요청 받을 때
        self.help_signal_sub = self.node.create_subscription(
            Bool,
            f'{self.other_namespace}/signal/help',
            self.get_help_signal_cb,
            10
        )

        self.help_coordinate_sub = self.node.create_subscription(
            Point,
            f'{self.other_namespace}/signal/coordinate',
            self.get_coordinate_signal_cb,
            10
        )
        # 상태
        self.latest_help = False
        self.pending_help = False
        self.last_point = None
        self._help_handled = False

        # [추가] YOLO 감지 결과를 받는 Subscriber
        self.detection_sub = self.node.create_subscription(
            String,
            'perception/detections',
            self.perception_callback,
            10
        )
        #####################################################################################
        # ---------------------------------------------------------
        # 1) Navigator
        # ---------------------------------------------------------
        self.navigator = TurtleBot4Navigator()
        # ✅ 과거 코드 호환: self.nav 를 쓰는 코드가 있어도 안 터지게
        self.nav = self.navigator
        
        self.initial_pose_robot2 = {
            "x": 3.710208,
            "y": 2.242908,
            "qz": 0.6096358,
            "qw": 0.7926816,
        }
        self.initial_pose_robot6 = {
                "x": -0.117279,
                "y": 0.019432,
                "qz": -0.771297,
                "qw": 0.636474,
        }
        self.predock_pose_robot2 = {
            "x": 3.70,
            "y": 2.00,
            "qz": 0.6096358,
            "qw": 0.7926816,
        }
        self.predock_pose_robot6 = {
                "x": -0.00918,
                "y": 0.004354,
                "qz": -0.771297,
                "qw": 0.636474,
        }


        # 도킹 상태 확인 (예외 방지)
        try:
            if not self.navigator.getDockedStatus():
                self.node.get_logger().info('[ActionLib] 도킹 상태가 아닙니다. 도킹을 시도합니다.')
                self.navigator.dock()
        except Exception as e:
            self.node.get_logger().warn(f"[ActionLib] Dock status check failed: {e}")

        # AMCL 초기 위치 설정
        if self.namespace == "/robot2":
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.initial_pose_robot2["x"]
            initial_pose.pose.position.y = self.initial_pose_robot2["y"]
            initial_pose.pose.orientation.z = self.initial_pose_robot2["qz"]
            initial_pose.pose.orientation.w = self.initial_pose_robot2["qw"]
        elif self.namespace == "/robot6":
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.initial_pose_robot6["x"]
            initial_pose.pose.position.y = self.initial_pose_robot6["y"]
            initial_pose.pose.orientation.z = self.initial_pose_robot6["qz"]
            initial_pose.pose.orientation.w = self.initial_pose_robot6["qw"]
        else:
            self.node.get_logger().warn(f"Unknown namespace: {self.namespace}")

        self.navigator.setInitialPose(initial_pose)

        # ❗가능하면 sleep 대신 nav stack ready 확인이 더 좋지만, 일단 유지
        time.sleep(1.0)
        self.node.get_logger().info('[ActionLib] 내비게이션 준비 완료.')

        # ---------------------------------------------------------
        # 2) Publishers (상대 토픽명: __ns 적용)
        # ---------------------------------------------------------
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.audio_pub = self.node.create_publisher(AudioNoteVector, 'cmd_audio', 10)

        # 서버로 보낼 토픽 ##########################
        self.fire_mode_pub = self.node.create_publisher(Bool, 'enable_fire_approach', 10)
        self.evac_pub = self.node.create_publisher(Bool, 'start_evacuation', 10)
        self.moving_pub = self.node.create_publisher(String, 'incident_status', 10)
        #########################################

        self.last_beep_time = 0.0
        
    # =========================================================
    # 공용: Nav2 완료 대기
    # =========================================================
    def wait_for_nav(self, timeout: float = 120.0, step_name: str = "nav"):
        start = time.time()
        while not self.navigator.isTaskComplete():
            if time.time() - start > timeout:
                self.node.get_logger().warn(f"[ActionLib] ⏰ timeout ({step_name}, {timeout}s). 다음 단계로 진행.")
                return False
            time.sleep(0.2)

        result = self.navigator.getResult()
        ok = (result == TaskResult.SUCCEEDED)
        self.node.get_logger().info(f"[ActionLib] {step_name} result={result} ok={ok}")
        return ok

    # =========================================================
    # Actions
    # =========================================================
    
    def action_1(self):
        """
        FullSequenceTest에서 code == 'afbfcn'일 때 호출.
        ns에 따라 robot2/robot6 이동 루트를 분기.
        """
        ns = self.node.get_namespace()
        self.node.get_logger().info(f"[Action1] start (ns={ns})")

        self.trigger_beep()

        # 1) Undock
        self.do_undock()
        # undock은 응답이 느릴 수 있으니 timeout 넉넉히
        self.wait_for_nav(timeout=25.0, step_name="undock")

        # 2) Namespace별 경로
        if ns == "/robot2":
            # robot2 -> 장소 A 루트 (a1->a2->a3)
            self.move_to_wp_a1(); self.wait_for_nav(step_name="wp_a1")
            self.move_to_wp_a2(); self.wait_for_nav(step_name="wp_a2")
            self.move_to_wp_a3(); self.wait_for_nav(step_name="wp_a3")
            

        elif ns == "/robot6":
            # robot6 -> 장소 B 루트 (b1->b2)
            self.move_to_wp_b1(); self.wait_for_nav(step_name="wp_b1")
            self.move_to_wp_b2(); self.wait_for_nav(step_name="wp_b2")

        else:
            self.node.get_logger().warn(
                f"[Action1] unknown namespace: {ns}. "
                "실행 시 --ros-args -r __ns:=/robot2 또는 /robot6 로 지정했는지 확인"
            )
            return

        self.node.get_logger().info("[Action1] done")

    def action_2(self):
        '''
        나머지 한대는 불 끄러 간 상태 이 로봇은 
        방에 가서 사람 있는지 확인 사람 있으면 
        evacuation mode 
        마지막 find 하면 비상구로 데려감 
        '''

        # 2. 탐색
        self.node.get_logger().info("Starting Mission.")
        target_found = self.go_to_A()
        
        if target_found:
            self.state = State.SEARCHING
            rotate_search = self.rotate_degree(-120.0)
        else:
            self.node.get_logger().info(f"A방 도착 실패{target_found}")

        if rotate_search:
            self.state = State.MOVINGAIN
            self.go_to_A_in()
        else:
            self.node.get_logger().info(f"A방 안쪽 도착 실패")

        if self.state == State.MOVINGAIN:
            self.rotate_degree(-120.0)
            self.state = State.FIND
        else:
            self.node.get_logger().info(f"탐색 실패")

        if self.state == State.FIND:
            self.guide_sequence()

    def action_3(self):
        '''
        a방에 불나고 b방에 서있는 사람 afbpcn
        혹은
        b방에 불나고 a방에 서 있는 사람 apbfcn
        '''
        # A방 들어가기 -> go to a
        # 회전 탐색 ( 첫번째방) -> 승호씨 코드 가져오기 
        # 두번째방 이동 후 회전탐색 -> go to a in 
        # 서있는 사람이 있으면 -> 회전 탐색 if 문 

        #회전탐색#####################################################
        
        found = self.spin_and_search_fire(timeout=15.0)  
        if found:
            self.moving_pub.publish(String(data="화재 접근 중"))
            # 찾은 상태에서 그대로 접근 시작
            self.action_approach_fire()
        else:
            self.node.get_logger().warn("❌ 화재를 찾지 못해 접근 단계를 건너뜁니다.")    
        

        ############################################################
        #--------------------------------------------------------
        # 대피 가이드 -> 
        # 일단 입구로 이동 
        # 입구 이동후 확인 
        # 3초마다 뒤돌기 도착할떄 까지 
        # 대피 다시키면 ? 도움요청 없으면 복도 순회, 도움요청 있으면 도와주러 가기 -> 우선순위는 사람 대피  
        #----------------------------------------------------------

        # 끝나면 순회 


        # TODO 객체 탐지 결과에 맞게 변수로 설정해야함 
        # 서있는 사람 탐지 결과 받아야 하므로 cls = stand 


    


    # =========================================================
    # Navigation Actions
    # =========================================================
    def do_undock(self):
        self.node.get_logger().info("Action: Undocking...")
        self.navigator.undock()

    def do_dock(self):
        self.node.get_logger().info("Action: Docking...")
        self.navigator.dock()

    def go_predock(self):
        if self.namespace == "/robot2":
            goal_pose = self.navigator.getPoseStamped([self.predock_pose_robot2["x"], self.predock_pose_robot2["y"]], TurtleBot4Directions.NORTH)
            self.navigator.startToPose(goal_pose)
        else:
            goal_pose = self.navigator.getPoseStamped([self.predock_pose_robot6["x"], self.predock_pose_robot6["y"]], TurtleBot4Directions.NORTH)
            self.navigator.startToPose(goal_pose)

    def move_to_wp_b1(self):
        self.node.get_logger().info("Action: Moving to WP_B1")
        goal_pose = self.navigator.getPoseStamped([0.6461, 2.7294], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal_pose)

    def move_to_wp_b2(self):
        self.node.get_logger().info("Action: Moving to WP_B2")
        goal_pose = self.navigator.getPoseStamped([2.0303, 2.1183], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal_pose)

    def move_to_wp_a1(self):
        self.node.get_logger().info("Action: Moving to WP_A1")
        goal_pose = self.navigator.getPoseStamped([3.9223, -0.3839], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal_pose)

    def move_to_wp_a2(self):
        self.node.get_logger().info("Action: Moving to WP_A2")
        goal_pose = self.navigator.getPoseStamped([3.3106, -1.7768], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal_pose)

    def move_to_wp_a3(self):
        self.node.get_logger().info("Action: Moving to WP_A3")
        goal_pose = self.navigator.getPoseStamped([3.1855, -3.7011], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal_pose)

    def perform_spin(self, duration=10.0):
        self.node.get_logger().info("Action: Spinning")
        self.navigator.spin(spin_dist=6.28, time_allowance=duration)

    def stop_robot(self):
        try:
            self.navigator.cancelTask()
        except Exception:
            pass
        self.cmd_vel_pub.publish(Twist())

    def is_nav_complete(self):
        return self.navigator.isTaskComplete()

    def is_nav_succeeded(self):
        return self.navigator.getResult() == TaskResult.SUCCEEDED
    
    def manual_rotate(self, angular_z: float):
        """cmd_vel 기반 즉시 회전 (rad/s)."""
        t = Twist()
        t.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(t)

    def manual_forward(self, linear_x: float):
        """cmd_vel 기반 즉시 전진 (m/s)."""
        t = Twist()
        t.linear.x = float(linear_x)
        self.cmd_vel_pub.publish(t)
    

        # [1]회전 함수
    def rotate_degree(self, degree:float):
        rad = math.radians(degree)
        self.node.get_logger().info(f"탐색을 위해 {degree}도 회전합니다...")

        # time_allowance: 10초 안에 못 돌면 실패 처리
        self.nav.spin(spin_dist=rad, time_allowance=10) 

        # [중요] 회전이 끝날 때까지 기다리는 루프
        while not self.nav.isTaskComplete():
            # 피드백을 받아와서 로그를 찍어도 됩니다 (선택사항)
            # feedback = self.navigator.getFeedback()
            pass  # 다 돌 때까지 대기

        # 결과 확인
        result = self.nav.getResult()
        
        # TaskResult.SUCCEEDED와 비교해야 하지만, 간단히 성공 로그 출력
        self.node.get_logger().info("회전 탐색 완료!")
        return True


    def go_to_A(self):
        # Start on dock
        if self.nav.getDockedStatus():
            self.node.get_logger().info("🔌 Undocking first...")
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
            self.node.get_logger().info("✅ A방 진입 완료 (성공)")
            return True
        elif result == TaskResult.CANCELED:
            self.node.get_logger().info("⚠️ 이동 취소됨")
            return False
        elif result == TaskResult.FAILED:
            self.node.get_logger().info("❌ 이동 실패 (경로 막힘 등)")
            return False
        else:
            return False


    def go_to_A_in(self):
        goal_pose = self.nav.getPoseStamped([3.1855, -3.7011], TurtleBot4Directions.SOUTH_EAST)
        self.nav.startToPose(goal_pose)


    def guide_sequence(self):
        self.node.get_logger().info("Step 3: Guiding to Evacuation Point...")
        # evac_pose = self.create_pose(self.evac_point)
        # self.nav.startToPose(evac_pose)
        # goal_pose = self.nav.getPoseStamped([0.972021, 0.383458], TurtleBot4Directions.NORTH)
        # self.nav.startToPose(goal_pose)
        self.nav.waitUntilNav2Active()


        pre_goal_pose = self.nav.getPoseStamped([3.92, -1.09], TurtleBot4Directions.SOUTH_EAST)
        # Set goal poses
        self.wait_for_nav(step_name="wp_b1")
        self.check_follower()

        goal_pose=self.nav.getPoseStamped([0.972021, 0.383458], TurtleBot4Directions.SOUTH_EAST) #대피소 좌표
        self.navigator.startToPose(pre_goal_pose)


        feed_back = self.nav.getFeedback()

        last_check_time = time.time()
        while feed_back.distance_remaining > 0.05:
            # 3초마다 확인
            if time.time() - last_check_time > 3.0:
                self.nav.cancelTask() # 잠시 멈춤
                
                self.check_follower()
                if not self.check_follower():
                    if not self.handle_lost_follower():
                        self.get_logger().error("사람을 완전히 놓쳤습니다. 미션 종료.")
                        return 
                
                self.get_logger().info("Resuming guide...")
                last_check_time = time.time()
                self.nav.startThroughPoses(goal_pose)
            print("현재 이동 중")

        
        self.node.get_logger().info("✅ Mission Complete.")
        self.state = State.IDLE


    # =========================================================
    # [NEW] 회전하며 화재 탐색 (발견 시 즉시 중단)
    # =========================================================
    def spin_and_search_fire(self, timeout=15.0):
        """
        제자리에서 회전하며 화재('fire')를 찾습니다.
        화재가 발견되면(self.target_fire is not None) 즉시 회전을 멈추고 True를 반환합니다.
        못 찾고 timeout이 지나면 False를 반환합니다.
        """
        self.node.get_logger().info("🔄 [Action] 회전하며 화재 탐색 시작...")
        
        start_time = time.time()
        
        # 1. 회전 명령 (cmd_vel)
        twist = Twist()
        twist.angular.z = 0.5  # 회전 속도 (너무 빠르면 감지 못함)
        
        while time.time() - start_time < timeout:
            # 화재 감지 확인
            if self.target_fire is not None:
                self.node.get_logger().info(f"🔥 [Action] 화재 발견! 회전 중단. (거리: {self.target_fire['dist']}m)")
                self.stop_robot() # 즉시 정지
                return True
            
            # 계속 회전
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
        self.stop_robot()
        self.node.get_logger().warn("⚠️ [Action] 탐색 실패 (시간 초과)")
        return False

    def action_approach_fire(self):
        """
        1. 화재 감지 대기
        2. 화면 중앙 맞추기 (회전) & 1.0m까지 접근 (전진)
        3. 도착 후 정지 및 30초 카운트다운
        """
        self.node.get_logger().info("🔥 [Action] 화재 접근 모드 시작. 화재를 찾습니다...")


        #  접근 제어 루프 (PID 제어와 유사)
        target_dist = 1.0  # 목표 거리 (미터)
        tolerance = 0.05   # 거리 허용 오차 (±5cm)
        center_tolerance = 20 # 픽셀 허용 오차

        while rclpy.ok():
            # 화재를 놓쳤을 경우 정지
            if self.target_fire is None:
                self.manual_forward(0.0)
                continue

            # 현재 데이터 가져오기
            cx = self.target_fire['cx']
            dist = self.target_fire['dist']

            # --- (1) 회전 제어 (화면 중앙 맞추기) ---
            center_x = self.img_width / 2
            error_x = center_x - cx
            
            # 오차가 크면 회전 (P제어)
            angular_z = 0.002 * error_x 
            angular_z = max(min(angular_z, 0.4), -0.4) # 속도 제한

            # 중앙에 거의 맞으면 회전 멈춤
            if abs(error_x) < center_tolerance:
                angular_z = 0.0

            # --- (2) 거리 제어 (1m 맞추기) ---
            linear_x = 0.0
            
            # 로봇이 대략 불을 바라보고 있을 때만 전진 (엉뚱한 곳으로 가는 것 방지)
            if abs(error_x) < 100:
                if dist > target_dist + tolerance:
                    linear_x = 0.15  # 천천히 전진
                elif dist < target_dist - tolerance:
                    linear_x = -0.05 # 너무 가까우면 살짝 후진
                else:
                    # 목표 거리에 도달함!
                    linear_x = 0.0
                    angular_z = 0.0
                    
                    # 완전 정지 명령
                    twist = Twist()
                    self.cmd_vel_pub.publish(twist)
                    self.node.get_logger().info(f"✅ [Action] 목표 지점 도착! (거리: {dist}m)")
                    break

            # 속도 명령 발행
            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.angular.z = float(angular_z)
            self.cmd_vel_pub.publish(twist)
            
            time.sleep(0.1)

        #  30초 대기 (진압 시뮬레이션)
        self.node.get_logger().info("🧯 [Timer] 화재 진압 작업을 시작합니다. (30초 대기)")
        
        start_time = time.time()
        while time.time() - start_time < 30.0:
            elapsed = int(time.time() - start_time)
            remaining = 30 - elapsed
            
            # 5초마다 로그 출력
            if elapsed > 0 and elapsed % 5 == 0:
                 self.node.get_logger().info(f"⏳ [Timer] 진압 중... {remaining}초 남음")
            
            time.sleep(1.0) # 1초씩 대기

        self.node.get_logger().info("🎉 [Timer] 30초 경과! 화재 진압 완료.")
        self.trigger_beep() # 완료 비프음

    # =========================================================
    # Utility
    # =========================================================
    ##############################################################################
    # 프리토킹 + 도킹 실패시 예외처리
    #################################################################################
    def stop_and_alarm_forever(self,reason: str = "", beep_period: float = 1.0):
        """
        도킹 실패/위험 상황: 로봇 정지 + 경고음 반복 + 무한 대기
        beep_period: 몇 초마다 삐-삐 할지
        """
        # 1) 정지/취소
        try:
            if hasattr(self, "navigator") and hasattr(self.navigator, "cancelTask"):
                self.navigator.cancelTask()
        except Exception:
            pass

        if reason:
            self.node.get_logger().error(reason)

        self.node.get_logger().error("[Fire] 도킹 실패 -> 안전 정지 + 경고음 반복(무한)")

        # 2) 경고음 반복 (삐-삐)
        next_beep = 0.0
        while rclpy.ok():
            now = time.time()
            if now >= next_beep:
                try:
                    # 삐-삐(두 번)
                    self.trigger_beep()
                    time.sleep(0.15)
                    self.trigger_beep()
                except Exception as be:
                    self.node.get_logger().warn(f"[Fire] beep 실패: {be}")
                next_beep = now + beep_period

            time.sleep(0.05)

    ##################################################################################
    def trigger_beep(self):
        now = time.time()
        if now - self.last_beep_time < 10.0:
            return
        self.last_beep_time = now

        msg = AudioNoteVector()

        # 환경에 따라 append 필드가 없을 수 있음
        try:
            msg.append = False
        except Exception:
            pass

        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
            AudioNote(frequency=440, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
        ]
        self.audio_pub.publish(msg)

    def send_help_signal(self, x, y):
        p = Point(x=float(x), y=float(y), z=0.0)
        self.help_pub.publish(p)

    def fire_search_and_chase(
        self,
        is_target_locked_fn,
        control_to_fire_fn,
        spin_duration: float = 10.0,
        chase_timeout: float = 60.0,
        rotate_speed: float = 0.3,
    ) -> bool:
        """
        정찰 회전 후 화재 추적 루프.
        - is_target_locked_fn: () -> bool  (외부 상태 판단 함수)
        - control_to_fire_fn: () -> bool   (접근 제어 1 step, 도착하면 True)
        """
        self.node.get_logger().info("[Fire] 정찰 회전")
        try:
            # 1) Spin
            self.perform_spin(duration=spin_duration)
            self.wait_for_nav(timeout=spin_duration + 20.0, step_name="spin")

            # 2) Chase loop
            self.node.get_logger().info("[Fire] 화재 추적 시작")
            start = time.time()

            fire_engage_start = None
            handover_sent = False

            while True:
                now = time.time()

                if now - start > chase_timeout:
                    raise TimeoutError("[Fire] 화재 접근 타임아웃")

                locked = is_target_locked_fn()

                if locked:
                    if fire_engage_start is None:
                        fire_engage_start = now
                        self.node.get_logger().info("[Fire] engage start")

                    if (not handover_sent) and (now - fire_engage_start >= 30.0):
                        hx, hy = 3.18, -3.70
                        self.send_help_point(hx, hy)
                        self.trigger_beep()
                        handover_sent = True

                    if control_to_fire_fn():
                        self.node.get_logger().info("[Fire] 화재 지점 도착!")
                        return True

                else:
                    self.manual_rotate(rotate_speed)
                    fire_engage_start = None
                    handover_sent = False

                time.sleep(0.1)

        except Exception as e:
            # 여기서 “추후 동작 실패”를 전부 잡음 (timeout 포함)
            self.node.get_logger().error(f"[Fire] 실패로 인해 복귀/도킹 수행: {e}")

            # (선택) 진행 중인 네비/회전 stop/cancel이 있다면 먼저 호출
            try:
                if hasattr(self, "navigator") and hasattr(self.navigator, "cancelTask"):
                    self.navigator.cancelTask()
            except Exception as ce:
                self.node.get_logger().warn(f"[Fire] cancelTask 실패: {ce}")

            # go_home + dock
            try:
                self.go_predock()
                self.wait_for_nav(timeout=30.0, step_name="predock") # 30초동안 프리도킹 위치로 이동 
            except Exception as he:
                self.node.get_logger().warn(f"[Fire] go_predock 실패: {he}")
                if hasattr(self, "navigator") and hasattr(self.navigator, "cancelTask"):
                    self.navigator.cancelTask()
            #도킹시도 
            try:
                if not self.navigator.getDockedStatus():
                    self.navigator.dock()

                # 도킹 완료 폴링(예: 30초)
                t0 = time.time()
                dock_timeout = 30.0
                while time.time() - t0 < dock_timeout:
                    if self.navigator.getDockedStatus():
                        self.node.get_logger().info("[Fire] 도킹 완료")
                        break
                    time.sleep(0.2)
                else:
                    self.stop_and_alarm_forever(self, reason="[Fire] 도킹 타임아웃(미도킹 상태)")

            except Exception as de:
                self.stop_and_alarm_forever(self, reason=f"[Fire] dock 실패: {de}")

            return False
        
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
        if self.is_stand_fresh(): # 1초동안 탐지됐나? TODO 추가 
            self.get_logger().info("✅ Follower confirmed (Visual)")
            is_following = True
        else:
            self.get_logger().warn("⚠️ No target visible!")
        
        self.nav.spin(spin_dist=3.14, time_allowance=10) # 혹은 -3.14
        while not self.nav.isTaskComplete():
            pass

        return is_following


        # self.node.get_logger().info("[Fire] 정찰 회전")
        # self.perform_spin(duration=spin_duration)
        # self.wait_for_nav(timeout=spin_duration + 20.0, step_name="spin")

        # self.node.get_logger().info("[Fire] 화재 추적 시작")
        # start = time.time()

        # fire_engage_start = None   # 화재 접근 시작 시각
        # handover_sent = False      # 교대 요청 1회만 보내기

        # while True:
        #     now = time.time()

        #     if now - start > chase_timeout:
        #         self.node.get_logger().error("[Fire] 화재 접근 타임아웃!")
        #         return False

        #     locked = is_target_locked_fn()

        #     # ============================
        #     # 🔥 화재 진압 중 상태
        #     # ============================
        #     if locked:

        #         # 화재 처음 잡은 순간 시간 기록
        #         if fire_engage_start is None:
        #             fire_engage_start = now
        #             self.node.get_logger().info("[Fire] engage start")

        #         # 🔴 여기에 넣는거다
        #         if (not handover_sent) and (now - fire_engage_start >= 30.0):

        #             # 교대 로봇이 와야 하는 위치
        #             hx, hy = 3.18, -3.70   # 또는 현재 로봇 위치

        #             self.send_help_point(hx, hy)
        #             self.trigger_beep()

        #             handover_sent = True

        #         # 실제 접근 제어
        #         if control_to_fire_fn():
        #             self.node.get_logger().info("[Fire] 화재 지점 도착!")
        #             return True

        #     # ============================
        #     # 🔄 타겟 못잡은 상태 → 탐색
        #     # ============================
        #     else:
        #         self.manual_rotate(rotate_speed)

        #         # 타겟 놓치면 진압 타이머 리셋
        #         fire_engage_start = None
        #         handover_sent = False

        #     time.sleep(0.1)

    def send_help_point(self, x, y):
        # 1) help=True 발행
        help_signal = Bool()
        help_signal.data = True
        self.help_signal_pub.publish(help_signal)

        # 2) 좌표 발행
        help_point = Point()
        help_point.x = float(x)
        help_point.y = float(y)
        help_point.z = 0.0
        self.help_coordinate_pub.publish(help_point)

        self.node.get_logger().warn(
            f"[Help] sent -> ({help_point.x:.3f}, {help_point.y:.3f})"
        )
    def get_help_signal_cb(self, msg: Bool):
        self.latest_help = bool(msg.data)

        self.node.get_logger().info(
            f"[Help RX] {self.other_namespace}/signal/help = {self.latest_help}"
        )

        if self.latest_help:
            self.pending_help = True
            self._try_handle_help()

    def get_coordinate_signal_cb(self, msg: Point):
        help_x, help_y = float(msg.x), float(msg.y)
        self.last_point = (help_x, help_y)

        self.node.get_logger().info(
            f"[Help RX] {self.other_namespace}/signal/coordinate = ({help_x:.3f},{help_y:.3f})"
        )

        self._try_handle_help()

    def _try_handle_help(self):
        if not self.pending_help:
            return

        if self.last_point is None:
            return

        if self._help_handled:
            return

        help_x, help_y = self.last_point

        self.node.get_logger().warn(
            f"[Help] handling -> go to ({help_x:.3f},{help_y:.3f})"
        )

        goal_pose = self.navigator.getPoseStamped(
            [help_x, help_y],
            TurtleBot4Directions.NORTH   # TODO: 방향 정책 필요
        )

        self.navigator.startToPose(goal_pose)

        self.pending_help = False
        self._help_handled = True

    def perception_callback(self, msg):
        #JSON 데이터를 파싱 parse

        try:
            detections = json.loads(msg.data)
            found = False
            
            # 감지된 물체 중 'fire'가 있는지 확인
            for obj in detections:
                if obj['class'] == 'fire':
                    self.target_fire = obj
                    found = True
                    break # 일단 하나만 잡습니다
            
            if not found:
                self.target_fire = None
                
        except Exception as e:
            self.node.get_logger().error(f"JSON 파싱 에러: {e}")
