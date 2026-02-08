# fight_fire/amr_actions.py
import json
import math
import time

import requests

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped
from std_msgs.msg import Bool, String
from irobot_create_msgs.msg import AudioNoteVector, AudioNote

from nav2_simple_commander.robot_navigator import TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions


class RobotActionLib:
    """
    Node 주입받는 라이브러리(ROS pub/sub 포함).
    nav 인스턴스는 self.nav 로 통일.
    """

    # help 이동 시 방향
    NAV_DIR_HELP = TurtleBot4Directions.NORTH

    def __init__(self, node: Node):
        self.node = node
        self.namespace = self.node.get_namespace()
        self.other_namespace = "/robot6" if self.namespace == "/robot2" else "/robot2"

        # ---- navigator ----
        self.nav = TurtleBot4Navigator()

        # ---- pubs ----
        self.cmd_vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.audio_pub = self.node.create_publisher(AudioNoteVector, "cmd_audio", 10)

        self.help_signal_pub = self.node.create_publisher(Bool, f"{self.namespace}/signal/help", 10)
        self.help_coordinate_pub = self.node.create_publisher(Point, f"{self.namespace}/signal/coordinate", 10)

        # ---- subs ----
        self.node.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._amcl_cb, 10)

        self.node.create_subscription(
            Bool, f"{self.other_namespace}/signal/help", self.get_help_signal_cb, 10
        )
        self.node.create_subscription(
            Point, f"{self.other_namespace}/signal/coordinate", self.get_coordinate_signal_cb, 10
        )

        self.node.create_subscription(String, "perception/detections", self.perception_callback, 10)

        # ---- state ----
        self.robot_x = None
        self.robot_y = None

        self.last_beep_time = 0.0

        self.target_fire = None
        self.target_stand = None

        self.img_width = 640

        self.latest_help = False
        self.pending_help = False
        self.last_point = None
        self._help_handled = False

        # predock (필요하면 바꿔)
        self.predock_pose_robot2 = {"x": 3.70, "y": 2.00}
        self.predock_pose_robot6 = {"x": -0.00918, "y": 0.004354}

        # cancel guard
        self._cancel_sent = False
        self._cancel_sent_ts = 0.0

    # ---------------------------
    # generic nav wait
    # ---------------------------
    def wait_for_nav(self, timeout: float = 120.0, step_name: str = "nav") -> bool:
        start = time.time()
        while not self.nav.isTaskComplete():
            if time.time() - start > timeout:
                self.node.get_logger().warn(f"[Nav] timeout ({step_name}, {timeout}s)")
                return False
            time.sleep(0.2)

        result = self.nav.getResult()
        ok = (result == TaskResult.SUCCEEDED)
        self.node.get_logger().info(f"[Nav] {step_name} result={result} ok={ok}")
        return ok

    # ---------------------------
    # stop/cancel
    # ---------------------------
    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())
        try:
            if self.nav.isTaskComplete():
                self._cancel_sent = False
                return
            now = time.time()
            if self._cancel_sent and (now - self._cancel_sent_ts < 1.0):
                return
            self.nav.cancelTask()
            self._cancel_sent = True
            self._cancel_sent_ts = now
        except Exception:
            pass

    # ---------------------------
    # beeps
    # ---------------------------
    def _beep_publish(self, freqs):
        msg = AudioNoteVector()
        try:
            msg.append = False
        except Exception:
            pass

        msg.notes = [
            AudioNote(
                frequency=int(f),
                max_runtime=Duration(seconds=0, nanoseconds=300_000_000).to_msg()
            )
            for f in freqs
        ]
        self.audio_pub.publish(msg)

    def trigger_beep(self):
        now = time.time()
        if now - self.last_beep_time < 1.0:
            return
        self.last_beep_time = now
        # do-re-mi-fa-sol
        self._beep_publish([261, 294, 330, 349, 392])

    def trigger_beep_err(self):
        now = time.time()
        if now - self.last_beep_time < 1.0:
            return
        self.last_beep_time = now
        self._beep_publish([880, 880, 880])

    def trigger_beep_ok(self):
        now = time.time()
        if now - self.last_beep_time < 1.0:
            return
        self.last_beep_time = now
        # do-mi-sol-do (up)
        self._beep_publish([523, 659, 784, 1046])

    # ---------------------------
    # dock/undock (bool return)
    # ---------------------------
    def action_undock(self) -> bool:
        try:
            # 이미 언도킹이면 성공
            if not self.nav.getDockedStatus():
                self.node.get_logger().info("[Dock] already undocked")
                return True

            self.node.get_logger().info("[Dock] undock start")
            self.nav.undock()
            ok = self.wait_for_nav(timeout=20.0, step_name="undock")
            if not ok:
                self.node.get_logger().warn("[Dock] undock failed")
                return False
            self.trigger_beep()
            return True
        except Exception as e:
            self.node.get_logger().warn(f"[Dock] undock exception: {e}")
            return False

    def action_dock(self) -> bool:
        try:
            # 이미 도킹이면 성공
            if self.nav.getDockedStatus():
                self.node.get_logger().info("[Dock] already docked")
                return True

            self.node.get_logger().info("[Dock] dock start")
            self.nav.dock()
            ok = self.wait_for_nav(timeout=30.0, step_name="dock")
            if not ok:
                self.node.get_logger().warn("[Dock] dock failed")
                return False
            self.trigger_beep()
            return True
        except Exception as e:
            self.node.get_logger().warn(f"[Dock] dock exception: {e}")
            return False

    # ---------------------------
    # simple moves (A/B/predock)
    # ---------------------------
    def go_predock(self) -> bool:
        if self.namespace == "/robot2":
            x, y = self.predock_pose_robot2["x"], self.predock_pose_robot2["y"]
        else:
            x, y = self.predock_pose_robot6["x"], self.predock_pose_robot6["y"]
        goal_pose = self.nav.getPoseStamped([x, y], TurtleBot4Directions.NORTH)
        self.nav.startToPose(goal_pose)
        return self.wait_for_nav(timeout=120.0, step_name="predock")

    def go_to_A(self) -> bool:
        self.nav.waitUntilNav2Active()

        poses = [
            self.nav.getPoseStamped([3.9223, -0.3839], TurtleBot4Directions.SOUTH_EAST),
            self.nav.getPoseStamped([3.3106, -1.7768], TurtleBot4Directions.SOUTH_EAST),
        ]
        self.nav.startThroughPoses(poses)
        return self.wait_for_nav(timeout=180.0, step_name="go_to_A")

    def go_to_B(self) -> bool:
        self.nav.waitUntilNav2Active()

        poses = [
            self.nav.getPoseStamped([0.6461, 2.7294], TurtleBot4Directions.NORTH),
            self.nav.getPoseStamped([2.0303, 2.1183], TurtleBot4Directions.NORTH),
        ]
        self.nav.startThroughPoses(poses)
        return self.wait_for_nav(timeout=180.0, step_name="go_to_B")

    # ---------------------------
    # help signal RX/TX
    # ---------------------------
    def send_help_point(self, x: float, y: float):
        s = Bool()
        s.data = True
        self.help_signal_pub.publish(s)

        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.0
        self.help_coordinate_pub.publish(p)

    def get_help_signal_cb(self, msg: Bool):
        self.latest_help = bool(msg.data)
        if self.latest_help:
            self.pending_help = True

    def get_coordinate_signal_cb(self, msg: Point):
        self.last_point = (float(msg.x), float(msg.y))

    # ---------------------------
    # perception parsing (fire + stand)
    # ---------------------------
    def perception_callback(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except Exception:
            return

        fire = None
        stand = None
        for obj in detections:
            c = obj.get("class")
            if c == "fire" and fire is None:
                fire = obj
            elif c == "stand" and stand is None:
                stand = obj
            if fire is not None and stand is not None:
                break

        self.target_fire = fire
        self.target_stand = stand

    # ---------------------------
    # AMCL pose
    # ---------------------------
    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.robot_x = float(p.x)
        self.robot_y = float(p.y)

    # ---------------------------
    # manual cmd_vel helpers
    # ---------------------------
    def manual_rotate(self, angular_z: float):
        t = Twist()
        t.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(t)

    def manual_forward(self, linear_x: float):
        t = Twist()
        t.linear.x = float(linear_x)
        self.cmd_vel_pub.publish(t)

    def manual_rotate_180(self):
        twist = Twist()
        twist.angular.z = 0.5
        duration = math.pi / 0.5  # ~6.28s
        start = time.time()
        while time.time() - start < duration and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.stop_robot()

    # ---------------------------
    # fire suppression mission
    # ---------------------------
    def fire_suppression_mission(self) -> bool:
        """
        Search(회전) -> Approach(정렬/접근) -> Suppression(5초 미감지 성공, 30초 지원요청)
        """
        self.node.get_logger().info("[Fire] mission start")

        # Phase 1: search (max 15s)
        if self.target_fire is None:
            start = time.time()
            tw = Twist()
            tw.angular.z = 0.5
            while time.time() - start < 15.0 and rclpy.ok():
                if self.target_fire is not None:
                    self.stop_robot()
                    break
                self.cmd_vel_pub.publish(tw)
                time.sleep(0.1)

            if self.target_fire is None:
                self.stop_robot()
                self.node.get_logger().warn("[Fire] search fail")
                return False

        # Phase 2: approach (max 60s)
        target_dist = 1.0
        dist_tol = 0.05
        center_tol = 20
        img_center_x = self.img_width / 2.0

        start_approach = time.time()
        while rclpy.ok():
            if time.time() - start_approach > 60.0:
                self.node.get_logger().warn("[Fire] approach timeout")
                self.stop_robot()
                return False

            if self.target_fire is None:
                self.manual_forward(0.0)
                time.sleep(0.1)
                continue

            cx = float(self.target_fire.get("cx", img_center_x))
            dist = float(self.target_fire.get("dist", 999.0))

            error_x = img_center_x - cx
            angular_z = max(min(0.002 * error_x, 0.4), -0.4)
            if abs(error_x) < center_tol:
                angular_z = 0.0

            linear_x = 0.0
            dist_err = dist - target_dist

            if abs(error_x) < 100:
                if dist > target_dist + dist_tol:
                    linear_x = 0.15
                elif dist < target_dist - dist_tol:
                    linear_x = -0.05
                else:
                    self.stop_robot()
                    break

            tw = Twist()
            tw.linear.x = float(linear_x)
            tw.angular.z = float(angular_z)
            self.cmd_vel_pub.publish(tw)
            time.sleep(0.1)

        # Phase 3: suppression monitor
        suppression_start = time.time()
        last_seen_time = time.time()
        help_sent = False
        try:
            requests.get("http://192.168.108.200:4000/gpio/high", timeout=1)
        except requests.exceptions.RequestException:
            pass

        while rclpy.ok():
            now = time.time()

            if self.target_fire is not None:
                last_seen_time = now

            # success if 5s no fire
            if now - last_seen_time > 5.0:
                self.node.get_logger().info("[Fire] success (5s no fire)")
                try:
                    requests.get("http://192.168.108.200:4000/gpio/low", timeout=1)
                except requests.exceptions.RequestException:
                    pass
                self.trigger_beep_ok()
                return True

            # help if 30s and still burning
            if (now - suppression_start > 30.0) and (not help_sent) and (now - last_seen_time < 1.0):
                self.node.get_logger().warn("[Fire] help request (30s)")
                try:
                    requests.get("http://192.168.108.200:4000/gpio/low", timeout=1)
                except requests.exceptions.RequestException:
                    pass
                if self.robot_x is not None and self.robot_y is not None:
                    self.send_help_point(self.robot_x, self.robot_y)
                self.trigger_beep()
                help_sent = True
                return False

            time.sleep(0.1)
            try:
                requests.get("http://192.168.108.200:4000/gpio/low", timeout=1)
            except requests.exceptions.RequestException:
                pass
        return False

    # ---------------------------
    # guide human sequence
    # ---------------------------
    def guide_human_sequence(self):
        """
        goal까지 Nav로 이동 중 5초마다 뒤 확인.
        """
        self.node.get_logger().info("[Guide] start")

        # evac point (바꿔)
        goal_x, goal_y = 0.972021, 0.383458
        target_pose = self.nav.getPoseStamped([goal_x, goal_y], TurtleBot4Directions.SOUTH_EAST)

        self.nav.startToPose(target_pose)
        last_check_time = time.time()

        while not self.nav.isTaskComplete() and rclpy.ok():
            if time.time() - last_check_time > 5.0:
                self.nav.cancelTask()
                self.stop_robot()

                self.manual_rotate_180()

                # wait until stand within 1.5m
                while rclpy.ok():
                    if self.target_stand is None:
                        time.sleep(0.5)
                        continue
                    dist = float(self.target_stand.get("dist", 0.0))
                    if dist != 0.0 and dist <= 1.5:
                        break
                    time.sleep(0.5)

                self.manual_rotate_180()

                self.nav.startToPose(target_pose)
                last_check_time = time.time()

            time.sleep(0.1)

        result = self.nav.getResult()
        if result != TaskResult.SUCCEEDED:
            raise RuntimeError("guide_human_sequence failed")
