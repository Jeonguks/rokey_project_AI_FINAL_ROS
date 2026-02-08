#!/usr/bin/env python3
import time
from typing import Callable, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Twist, Point
from std_msgs.msg import Bool, String
from irobot_create_msgs.msg import AudioNoteVector, AudioNote

from nav2_simple_commander.robot_navigator import TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


class RobotActionLib:
    """
    RobotActionLib = 실행기(Executor).
    - 상위(FullSequenceTest)가 의사결정/우선순위/프리엠션을 담당
    - 여기서는 nav/cmd_vel primitive 제공 + 도움요청 publish만 제공
    """

    def __init__(self, node: Node):
        self.node = node
        self.namespace = self.node.get_namespace()

        # other ns는 FullSequenceTest에서 판단해도 되지만, 발신용 로그 등에 사용 가능
        self.other_namespace = "/robot6" if self.namespace == "/robot2" else "/robot2"

        # -------------------------
        # HELP publish only (구독 없음)
        # -------------------------
        self.help_signal_pub = self.node.create_publisher(
            Bool, f"{self.namespace}/signal/help", 10
        )
        self.help_coordinate_pub = self.node.create_publisher(
            Point, f"{self.namespace}/signal/coordinate", 10
        )

        # -------------------------
        # Nav2
        # -------------------------
        self.navigator = TurtleBot4Navigator()
        self.nav = self.navigator  # 호환 alias

        # -------------------------
        # Pubs
        # -------------------------
        self.cmd_vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.audio_pub = self.node.create_publisher(AudioNoteVector, "cmd_audio", 10)

        # (선택) 서버로 보낼 토픽들
        self.fire_mode_pub = self.node.create_publisher(Bool, "enable_fire_approach", 10)
        self.evac_pub = self.node.create_publisher(Bool, "start_evacuation", 10)
        self.moving_pub = self.node.create_publisher(String, "incident_status", 10)

        self.last_beep_time = 0.0

        # -------------------------
        # Pose presets
        # -------------------------
        self.initial_pose_robot2 = {"x": 3.710208, "y": 2.242908, "qz": 0.6096358, "qw": 0.7926816}
        self.initial_pose_robot6 = {"x": -0.117279, "y": 0.019432, "qz": -0.771297, "qw": 0.636474}

        self.predock_pose_robot2 = {"x": 3.70, "y": 2.00, "qz": 0.6096358, "qw": 0.7926816}
        self.predock_pose_robot6 = {"x": -0.00918, "y": 0.004354, "qz": -0.771297, "qw": 0.636474}

        # -------------------------
        # 초기 위치 설정 (AMCL)
        # -------------------------
        self._set_initial_pose()

        # Nav2 준비 대기 (가능하면 waitUntilNav2Active가 더 낫지만, 환경 따라 다름)
        time.sleep(1.0)
        self.node.get_logger().info("[ActionLib] Navigation ready.")

    # =========================================================
    # Init helpers
    # =========================================================
    def _set_initial_pose(self):
        ns = self.namespace
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        if ns == "/robot2":
            p = self.initial_pose_robot2
        elif ns == "/robot6":
            p = self.initial_pose_robot6
        else:
            self.node.get_logger().warn(f"[ActionLib] Unknown namespace: {ns} (expected /robot2 or /robot6)")
            p = self.initial_pose_robot2

        initial_pose.pose.position.x = float(p["x"])
        initial_pose.pose.position.y = float(p["y"])
        initial_pose.pose.orientation.z = float(p["qz"])
        initial_pose.pose.orientation.w = float(p["qw"])

        self.navigator.setInitialPose(initial_pose)

    # =========================================================
    # Common: cancel/stop
    # =========================================================
    def cancel_current_task(self):
        """Preempt-safe: Nav2 cancel + cmd_vel stop"""
        try:
            if hasattr(self.navigator, "cancelTask"):
                self.navigator.cancelTask()
        except Exception as e:
            self.node.get_logger().warn(f"[ActionLib] cancelTask failed: {e}")

        self.cmd_vel_pub.publish(Twist())

    def stop_robot(self):
        """Alias"""
        self.cancel_current_task()

    # =========================================================
    # Nav wait
    # =========================================================
    def wait_for_nav(self, timeout: float = 120.0, step_name: str = "nav") -> bool:
        start = time.time()
        while not self.navigator.isTaskComplete():
            if time.time() - start > timeout:
                self.node.get_logger().warn(f"[ActionLib] ⏰ timeout ({step_name}, {timeout}s)")
                return False
            time.sleep(0.2)

        result = self.navigator.getResult()
        ok = (result == TaskResult.SUCCEEDED)
        self.node.get_logger().info(f"[ActionLib] {step_name} result={result} ok={ok}")
        return ok

    # =========================================================
    # Beep
    # =========================================================
    def trigger_beep(self):
        now = time.time()
        if now - self.last_beep_time < 3.0:
            return
        self.last_beep_time = now

        msg = AudioNoteVector()
        try:
            msg.append = False
        except Exception:
            pass

        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(seconds=0, nanoseconds=300_000_000).to_msg()),
            AudioNote(frequency=440, max_runtime=Duration(seconds=0, nanoseconds=300_000_000).to_msg()),
        ]
        self.audio_pub.publish(msg)

    # =========================================================
    # HELP publish
    # =========================================================
    def send_help_point(self, x: float, y: float):
        """Publish help(True) + coordinate"""
        self.help_signal_pub.publish(Bool(data=True))
        self.help_coordinate_pub.publish(Point(x=float(x), y=float(y), z=0.0))
        self.node.get_logger().warn(f"[Help] sent -> ({x:.3f}, {y:.3f})")

    # =========================================================
    # Go-to helpers
    # =========================================================
    def go_to_help_point(self, x: float, y: float, timeout: float = 120.0) -> bool:
        goal = self.navigator.getPoseStamped([float(x), float(y)], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal)
        return self.wait_for_nav(timeout=timeout, step_name="help_goal")

    def go_predock(self):
        if self.namespace == "/robot2":
            p = self.predock_pose_robot2
        else:
            p = self.predock_pose_robot6
        goal = self.navigator.getPoseStamped([p["x"], p["y"]], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal)

    def do_undock(self):
        self.node.get_logger().info("[ActionLib] Undocking...")
        self.navigator.undock()

    def do_dock(self):
        self.node.get_logger().info("[ActionLib] Docking...")
        self.navigator.dock()

    # =========================================================
    # Waypoints
    # =========================================================
    def move_to_wp_b1(self):
        goal = self.navigator.getPoseStamped([0.6461, 2.7294], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal)

    def move_to_wp_b2(self):
        goal = self.navigator.getPoseStamped([2.0303, 2.1183], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal)

    def move_to_wp_a1(self):
        goal = self.navigator.getPoseStamped([3.9223, -0.3839], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal)

    def move_to_wp_a2(self):
        goal = self.navigator.getPoseStamped([3.3106, -1.7768], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal)

    def move_to_wp_a3(self):
        goal = self.navigator.getPoseStamped([3.1855, -3.7011], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal)

    def perform_spin(self, duration: float = 10.0):
        # spin_dist=6.28(rad) ~ 360도
        self.navigator.spin(spin_dist=6.28, time_allowance=duration)

    def manual_rotate(self, angular_z: float):
        t = Twist()
        t.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(t)

    # =========================================================
    # Actions
    # =========================================================
    def action_1(self):
        """
        code == afbfcn: 양쪽 방 화재, 각자 방 이동 루트 분기
        """
        ns = self.namespace
        self.node.get_logger().info(f"[Action1] start (ns={ns})")

        self.trigger_beep()

        # 1) Undock
        self.do_undock()
        self.wait_for_nav(timeout=25.0, step_name="undock")

        # 2) Route
        if ns == "/robot2":
            self.move_to_wp_a1(); self.wait_for_nav(step_name="wp_a1")
            self.move_to_wp_a2(); self.wait_for_nav(step_name="wp_a2")
            self.move_to_wp_a3(); self.wait_for_nav(step_name="wp_a3")
        elif ns == "/robot6":
            self.move_to_wp_b1(); self.wait_for_nav(step_name="wp_b1")
            self.move_to_wp_b2(); self.wait_for_nav(step_name="wp_b2")
        else:
            self.node.get_logger().warn(f"[Action1] unknown namespace: {ns}")
            return

        self.node.get_logger().info("[Action1] done")

    def action_2(self):
        # TODO
        self.node.get_logger().info("[Action2] TODO")
        return

    # =========================================================
    # Fire search + chase (supports preempt)
    # =========================================================
    def fire_search_and_chase(
        self,
        is_target_locked_fn: Callable[[], bool],
        control_to_fire_fn: Callable[[], bool],
        should_preempt_fn: Optional[Callable[[], bool]] = None,
        spin_duration: float = 10.0,
        chase_timeout: float = 60.0,
        rotate_speed: float = 0.3,
    ) -> bool:
        """
        - should_preempt_fn(): True면 즉시 중단(preempt)
        """
        try:
            self.node.get_logger().info("[Fire] scouting spin")
            self.perform_spin(duration=spin_duration)
            self.wait_for_nav(timeout=spin_duration + 20.0, step_name="spin")

            self.node.get_logger().info("[Fire] chase loop start")
            start = time.time()

            fire_engage_start = None
            handover_sent = False

            while rclpy.ok():
                if should_preempt_fn and should_preempt_fn():
                    self.node.get_logger().warn("[Fire] PREEMPT detected -> abort chase")
                    self.cancel_current_task()
                    return False

                now = time.time()
                if now - start > chase_timeout:
                    raise TimeoutError("[Fire] chase timeout")

                locked = is_target_locked_fn()

                if locked:
                    if fire_engage_start is None:
                        fire_engage_start = now
                        self.node.get_logger().info("[Fire] engage start")

                    # 30초 경과 시 도움 요청 1회
                    if (not handover_sent) and (now - fire_engage_start >= 30.0):
                        hx, hy = 3.18, -3.70
                        self.send_help_point(hx, hy)
                        self.trigger_beep()
                        handover_sent = True

                    if control_to_fire_fn():
                        self.node.get_logger().info("[Fire] arrived fire point!")
                        return True
                else:
                    self.manual_rotate(rotate_speed)
                    fire_engage_start = None
                    handover_sent = False

                time.sleep(0.1)

        except Exception as e:
            self.node.get_logger().error(f"[Fire] failed: {e}")
            self.cancel_current_task()
            return False
