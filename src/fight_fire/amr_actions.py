#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Twist, Point, PoseWithCovarianceStamped
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
        self.amcl_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self._amcl_cb,
            10
        )
        
        # 상태
        self.latest_help = False
        self.pending_help = False
        self.last_point = None
        self._help_handled = False

        self._cancel_sent = False
        self._cancel_sent_ts = 0.0

        ########################################
        self.robot_x = None
        self.robot_y = None
        ############################################################
        # [추가] YOLO 감지 결과를 받는 Subscriber
        self.detection_sub = self.node.create_subscription(
            String,
            'perception/detections',
            self.perception_callback,
            10
        )
        #####################################################################################

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
    # Navigation Actions
    # =========================================================

    def go_predock(self):
        if self.namespace == "/robot2":
            goal_pose = self.navigator.getPoseStamped([self.predock_pose_robot2["x"], self.predock_pose_robot2["y"]], TurtleBot4Directions.NORTH)
            self.navigator.startToPose(goal_pose)
        else:
            goal_pose = self.navigator.getPoseStamped([self.predock_pose_robot6["x"], self.predock_pose_robot6["y"]], TurtleBot4Directions.NORTH)
            self.navigator.startToPose(goal_pose)

    # def move_to_wp_b1(self):
    #     self.node.get_logger().info("Action: Moving to WP_B1")
    #     goal_pose = self.navigator.getPoseStamped([0.6461, 2.7294], TurtleBot4Directions.NORTH)
    #     self.navigator.startToPose(goal_pose)

    # def move_to_wp_b2(self):
    #     self.node.get_logger().info("Action: Moving to WP_B2")
    #     goal_pose = self.navigator.getPoseStamped([2.0303, 2.1183], TurtleBot4Directions.NORTH)
    #     self.navigator.startToPose(goal_pose)

    # def move_to_wp_a1(self):
    #     self.node.get_logger().info("Action: Moving to WP_A1")
    #     goal_pose = self.navigator.getPoseStamped([3.9223, -0.3839], TurtleBot4Directions.NORTH)
    #     self.navigator.startToPose(goal_pose)

    # def move_to_wp_a2(self):
    #     self.node.get_logger().info("Action: Moving to WP_A2")
    #     goal_pose = self.navigator.getPoseStamped([3.3106, -1.7768], TurtleBot4Directions.NORTH)
    #     self.navigator.startToPose(goal_pose)

    # def move_to_wp_a3(self):
    #     self.node.get_logger().info("Action: Moving to WP_A3")
    #     goal_pose = self.navigator.getPoseStamped([3.1855, -3.7011], TurtleBot4Directions.NORTH)
    #     self.navigator.startToPose(goal_pose)

    def perform_spin(self, duration=10.0):
        self.node.get_logger().info("Action: Spinning")
        self.navigator.spin(spin_dist=6.28, time_allowance=duration)

    def stop_robot(self):
        # 1) cmd_vel 정지
        self.cmd_vel_pub.publish(Twist())

        # 2) Nav task cancel은 "한 번만"
        try:
            if self.navigator.isTaskComplete():
                # task가 끝났으면 cancel 가드 리셋
                self._cancel_sent = False
                return

            # task 진행중인데, 이미 cancel을 최근에 보냈으면 또 보내지 않음
            now = time.time()
            if self._cancel_sent and (now - self._cancel_sent_ts < 1.0):
                return

            self.navigator.cancelTask()
            self._cancel_sent = True
            self._cancel_sent_ts = now

        except Exception:
            pass

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


    def go_to_A(self)->Bool:
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
            #feedback = self.nav.getFeedback()
            #self.get_logger().info(f'이동 중... 남은 거리: {feedback.distance_remaining}')
            
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

    def go_to_A_enterence(self):
        goal_pose = self.nav.getPoseStamped([3.92, -1.09], TurtleBot4Directions.SOUTH_EAST)
        self.nav.startToPose(goal_pose)

    def go_to_B(self):
         # Wait for Nav2
        self.nav.waitUntilNav2Active()
        # Set goal poses
        goal_pose = []
        goal_pose.append(self.nav.getPoseStamped([0.6461, 2.7294], TurtleBot4Directions.NORTH))
        goal_pose.append(self.nav.getPoseStamped([2.0303, 2.1183], TurtleBot4Directions.NORTH))

        # Navigate through poses
        self.nav.startThroughPoses(goal_pose)
    
        # 2. 도착할 때까지 기다리는 루프 (가장 중요!)
        while not self.nav.isTaskComplete():
            #feedback = self.nav.getFeedback()
            #self.get_logger().info(f'이동 중... 남은 거리: {feedback.distance_remaining}')
            
            # 0.1초 정도 대기하며 루프 반복 (CPU 과부하 방지)
            # rclpy.spin_once() 같은 처리가 필요할 수도 있음 (구조에 따라 다름)
            pass

        # 3. 루프가 끝나면(도착하거나 취소되면) 최종 결과 확인
        result = self.nav.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.node.get_logger().info("✅ B방 진입 완료 (성공)")
            return True
        elif result == TaskResult.CANCELED:
            self.node.get_logger().info("⚠️ 이동 취소됨")
            return False
        elif result == TaskResult.FAILED:
            self.node.get_logger().info("❌ 이동 실패 (경로 막힘 등)")
            return False
        else:
            return False       

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
    # [통합] 화재 탐색 -> 정밀 접근 -> 진압/지원요청 (All-in-One)
    # =========================================================
    def fire_suppression_mission(self):
        """
        1. Search: 회전하며 불 찾기
        2. Approach: 중앙 정렬 및 1.0m 접근 (PID 제어)
        3. Suppression: 5초간 불 안 보이면 성공, 30초 경과 시 지원 요청
        """
        self.node.get_logger().info("🚀 [Mission] 화재 진압 통합 미션 시작")

        # -----------------------------------------------------
        # Phase 1: Search (회전 탐색)
        # -----------------------------------------------------
        # 만약 이미 불이 보이면 탐색 생략
        if self.target_fire is None:
            self.node.get_logger().info("🔄 [Step 1] 화재 탐색 중 (회전)...")
            start_search = time.time()
            twist = Twist()
            twist.angular.z = 0.5
            
            found = False
            while time.time() - start_search < 15.0: # 15초 제한
                if self.target_fire is not None:
                    self.stop_robot()
                    found = True
                    self.node.get_logger().info(f"🔥 화재 발견! (거리: {self.target_fire['dist']}m)")
                    break
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            
            if not found:
                self.stop_robot()
                self.node.get_logger().warn("❌ 탐색 실패: 화재를 찾지 못했습니다.")
                return False # 미션 실패

        # -----------------------------------------------------
        # Phase 2: Approach (정밀 접근 - PID 제어)
        # -----------------------------------------------------
        self.node.get_logger().info("🏃 [Step 2] 화재 지점으로 정밀 접근 (1.0m 목표)")
        
        # 제어 파라미터
        target_dist = 1.0       # 목표 거리 (m)
        dist_tolerance = 0.05   # 거리 오차 허용 범위 (m)
        center_tolerance = 20   # 픽셀 오차 허용 범위
        img_center_x = self.img_width / 2

        # 접근 타임아웃 (무한 루프 방지)
        start_approach = time.time()
        
        while rclpy.ok():
            # 타임아웃 체크 (60초 동안 못 가면 실패)
            if time.time() - start_approach > 60.0:
                self.node.get_logger().error("❌ 접근 시간 초과!")
                return False

            # 도중에 불을 놓치면? -> 잠시 정지하고 대기 (다시 보일 수도 있음)
            if self.target_fire is None:
                self.manual_forward(0.0)
                time.sleep(0.1)
                continue

            # 데이터 추출
            cx = self.target_fire['cx']
            dist = self.target_fire['dist']

            # --- 회전 제어 (화면 중앙 맞추기) ---
            error_x = img_center_x - cx
            angular_z = 0.002 * error_x
            # 속도 제한 (-0.4 ~ 0.4)
            angular_z = max(min(angular_z, 0.4), -0.4)

            if abs(error_x) < center_tolerance:
                angular_z = 0.0

            # --- 거리 제어 (1.0m 맞추기) ---
            linear_x = 0.0
            dist_error = dist - target_dist

            # 중앙이 어느 정도 맞았을 때만 전진 (안 그러면 엉뚱한 데로 감)
            if abs(error_x) < 100:
                if dist > target_dist + dist_tolerance:
                    linear_x = 0.15  # 전진
                elif dist < target_dist - dist_tolerance:
                    linear_x = -0.05 # 너무 가까우면 후진
                else:
                    # 거리도 맞고, 각도도 맞으면 도착!
                    self.stop_robot()
                    self.node.get_logger().info(f"✅ [Step 2] 도착 완료! (거리: {dist}m)")
                    break # Phase 3로 이동

            # 명령 발행
            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.angular.z = float(angular_z)
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        # -----------------------------------------------------
        # Phase 3: Suppression (진압 확인 및 지원 요청)
        # -----------------------------------------------------
        self.node.get_logger().info("🧯 [Step 3] 진압 작업 및 감시 시작...")
        
        suppression_start = time.time()
        last_seen_time = time.time()
        help_sent = False

        while rclpy.ok():
            now = time.time()

            # 1. 불이 보이는지 체크
            if self.target_fire is not None:
                last_seen_time = now # 불이 보이면 시간 갱신
            
            # 2. 소화 완료 판단 (5초 동안 불이 안 보임)
            if now - last_seen_time > 5.0:
                self.node.get_logger().info("✨ [Success] 화재 소화 완료! (5초간 미감지)")
                self.trigger_beep()
                return True

            # 3. 지원 요청 (30초 지났는데 아직 불이 안 꺼짐)
            # (불이 마지막으로 보인 시간이 최근 1초 이내여야 함 = 아직 불타는 중)
            if (now - suppression_start > 30.0) and (not help_sent) and (now - last_seen_time < 1.0):
                self.node.get_logger().warn("🚨 [Help] 30초 경과! 진압 실패. 지원 요청 전송.")
                
                # 현재 내 위치 보내기
                self.send_help_point(self.robot_x, self.robot_y)
                self.trigger_beep()
                help_sent = True
                
                # 지원 요청 후에는 복귀를 할지, 계속 대기할지 정책 결정
                # 여기서는 일단 미션 실패로 간주하고 복귀 절차 실행
                self.node.get_logger().info("⚠️ 지원 요청 후 복귀 절차로 넘어갑니다.")
                return False

            time.sleep(0.1)
  
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

        # msg.notes = [
        #     AudioNote(frequency=880, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
        #     AudioNote(frequency=440, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
        # ]
        msg.notes = [
            AudioNote(frequency=261, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
            AudioNote(frequency=294, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
            AudioNote(frequency=330, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
            AudioNote(frequency=349, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
            AudioNote(frequency=392, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
        ]
        self.audio_pub.publish(msg)

    def trigger_beep_err(self):
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
            AudioNote(frequency=880, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
            AudioNote(frequency=880, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),

        ]

        self.audio_pub.publish(msg)

    def trigger_beep_ok(self):
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
            AudioNote(frequency=523, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
            AudioNote(frequency=659, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
            AudioNote(frequency=784, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),
            AudioNote(frequency=1046, max_runtime=Duration(seconds=0, nanoseconds=300000000).to_msg()),

        ]

        self.audio_pub.publish(msg)
        
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


    def send_help_point(self, x:float, y:float):


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
        # help가 pending 아니면 패스
        if not self.pending_help:
            return
        if self.last_point is None:
            return
        if self._help_handled:
            return

        # ✅ robot6는 action_3에서만 처리 (콜백에서 startToPose 금지)
        if self.namespace == "/robot6":
            self.node.get_logger().warn(
                f"[Help] (robot6) 도움요청 예약됨 -> ({self.last_point[0]:.3f},{self.last_point[1]:.3f})"
            )
            return

        # (robot2는 기존처럼 즉시 처리하고 싶으면 여기서 처리)
        help_x, help_y = self.last_point
        self.node.get_logger().warn(f"[Help] handling -> go to ({help_x:.3f}, {help_y:.3f})")

        goal_pose = self.navigator.getPoseStamped([help_x, help_y], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal_pose)

        self.pending_help = False
        self._help_handled = True

    def _help_interrupt_requested_robot6(self) -> bool:
        return (self.namespace == "/robot6") and bool(self.pending_help) and (self.last_point is not None) and (not self._help_handled)

    def _handle_help_interrupt_robot6(self, step_name: str = "help_interrupt") -> bool:
        """
        robot6: action_3 수행 중 도움 요청 오면 즉시 현재 행동 중단하고 좌표로 이동
        """
        if not self._help_interrupt_requested_robot6():
            return False

        hx, hy = self.last_point
        self.node.get_logger().warn(f"[robot6:{step_name}] 도움요청 -> ({hx:.3f}, {hy:.3f})로 즉시 이동")

        # 1) cmd_vel 루프가 있으면 멈춤
        self.stop_robot()

        # 2) Nav2 태스크 취소(진행중이면)
        try:
            self.navigator.cancelTask()
        except Exception:
            pass

        # 3) help goal 실행
        goal_pose = self.navigator.getPoseStamped([hx, hy], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal_pose)

        ok = self.wait_for_nav(timeout=120.0, step_name="robot6_help_goal")

        # 4) 플래그 정리
        self.pending_help = False
        self._help_handled = True
        self.latest_help = False

        self.node.get_logger().warn(f"[robot6:{step_name}] help_goal 도착 ok={ok}")
        return ok

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

    def action_undock(self):
        try:
            if not self.nav.getDockedStatus():
                self.node.get_logger().info("[ActionLib] 이미 언도킹 입니다. 다음 액션 시작")
                return
            
            self.node.get_logger().info("[ActionLib] 도킹 해제 시작")
            self.nav.undock()
            ok = self.wait_for_nav(timeout=15.0, step_name="undock")
            if not ok:
                self.node.get_logger().warn(f"[ActionLib] 언도킹 실패")
                raise RuntimeError("Undock failed")
            self.trigger_beep()

        except Exception as e:
            self.node.get_logger().warn(f"[ActionLib] 언도킹 실패, 에러: {e}")

    def action_dock(self):
        try:
            if self.navigator.getDockedStatus():
                self.node.get_logger().info("[ActionLib] 이미 도킹 입니다. 다음 액션 시작")
                return
            
            self.node.get_logger().info('[ActionLib] 도킹 상태가 아닙니다. 도킹을 시도합니다.')
            self.navigator.dock()
            ok = self.wait_for_nav(timeout=15.0, step_name="dock")
            if not ok:
                self.node.get_logger().warn(f"[ActionLib] 도킹 실패")
                raise RuntimeError("Dock failed")
            self.trigger_beep()

        except Exception as e:
            self.node.get_logger().warn(f"[ActionLib] 도킹 실패, 에러: {e}")


    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.robot_x = float(p.x)
        self.robot_y = float(p.y)