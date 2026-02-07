#!/usr/bin/env python3
import time

from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Twist, Point
from std_msgs.msg import Bool
from irobot_create_msgs.msg import AudioNoteVector, AudioNote

from nav2_simple_commander.robot_navigator import TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


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



        # ---------------------------------------------------------
        # 1) Navigator
        # ---------------------------------------------------------
        self.navigator = TurtleBot4Navigator()
        
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
            "x": 3.16,
            "y": 1.87,
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

        # 전역 토픽 유지(원하면 상대 토픽으로 변경 가능)
        self.help_pub = self.node.create_publisher(Point, '/signal/rotation6', 10)

        self.fire_mode_pub = self.node.create_publisher(Bool, 'enable_fire_approach', 10)
        self.evac_pub = self.node.create_publisher(Bool, 'start_evacuation', 10)

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

    def stop_robot(self):
        # 기존 유지 + cmd_vel 0
        try:
            self.navigator.cancelTask()
        except Exception:
            pass
        self.cmd_vel_pub.publish(Twist())
    # =========================================================
    # Utility
    # =========================================================
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
                # go_home이 startToPose 기반이면 완료 대기 필요
                # 너 코드에 wait_for_nav 같은 게 있으면 그걸로 대기
                if hasattr(self, "wait_for_nav"):
                    self.wait_for_nav(timeout=120.0, step_name="go_predock")
            except Exception as he:
                self.node.get_logger().warn(f"[Fire] go_predock 실패: {he}")

            try:
                # 도킹 상태 확인 후 도킹
                if not self.navigator.getDockedStatus():
                    self.navigator.dock()
            except Exception as de:
                self.node.get_logger().warn(f"[Fire] dock 실패: {de}")

            return False
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