#!/usr/bin/env python3
import threading
import time
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Bool

from fight_fire.amr_actions import RobotActionLib


class FullSequenceTest(Node):
    def __init__(self):
        super().__init__("full_sequence_test_node")

        # -------------------------
        # namespace resolve
        # -------------------------
        self.namespace = self.get_namespace()
        self.other_namespace = "/robot6" if self.namespace == "/robot2" else "/robot2"

        # -------------------------
        # ActionLib (DI)
        # -------------------------
        self.actions = RobotActionLib(self)

        # -------------------------
        # Mission state
        # -------------------------
        self.is_mission_running = False
        self.target_locked = False
        self.latest_fire_info = None

        # trigger code store
        self._code_lock = threading.Lock()
        self._latest_code = None
        self._latest_code_time = 0.0

        # debounce
        self._last_handled_code = None
        self._last_handled_time = 0.0
        self.code_debounce_sec = 2.0

        # -------------------------
        # HELP preempt (최우선)
        # -------------------------
        self._help_event = threading.Event()
        self._help_lock = threading.Lock()
        self._help_point = None  # (x,y)

        # -------------------------
        # Subscriptions
        # -------------------------
        self.create_subscription(String, "/webcam_detected", self.trigger_callback, 10)
        self.create_subscription(String, "perception/detections", self.perception_callback, 10)

        # 상대 로봇 help 요청/좌표
        self.create_subscription(Bool, f"{self.other_namespace}/signal/help", self._on_help_signal, 10)
        self.create_subscription(Point, f"{self.other_namespace}/signal/coordinate", self._on_help_point, 10)

        self.get_logger().info("✅ FullSequenceTest ready.")
        self.get_logger().info(f"my ns={self.namespace}, other ns={self.other_namespace}")

        # mission loop thread
        self._mission_thread = threading.Thread(target=self.run_mission_sequence, daemon=True)
        self._mission_thread.start()

    # ---------------------------------------------------------
    # Trigger: JSON -> code
    # ---------------------------------------------------------
    def map_detection_to_code(self, detection_list):
        if not detection_list:
            return "n"
        s = set(detection_list)
        if "fire" in s:
            return "f"
        if ("stand" in s) or ("down" in s):
            return "p"
        return "n"

    def build_code_from_detection(self, data: dict) -> str:
        a_code = self.map_detection_to_code(data.get("class_a_detection", []))
        b_code = self.map_detection_to_code(data.get("class_b_detection", []))
        c_code = self.map_detection_to_code(data.get("class_c_detection", []))
        return f"a{a_code}b{b_code}c{c_code}"

    def trigger_callback(self, msg: String):
        raw = (msg.data or "").strip()
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().warn(f"[Trigger RX] invalid JSON: {raw[:120]}")
            return

        code = self.build_code_from_detection(data)

        with self._code_lock:
            self._latest_code = code
            self._latest_code_time = time.time()

        self.get_logger().info(f"[Trigger RX] json -> code='{code}'")

    # ---------------------------------------------------------
    # Perception: fire lock info (only during mission)
    # ---------------------------------------------------------
    def perception_callback(self, msg: String):
        if not self.is_mission_running:
            return

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        fire_obj = None
        for obj in data:
            if obj.get("class") == "fire":
                fire_obj = obj
                break

        if fire_obj:
            self.target_locked = True
            self.latest_fire_info = fire_obj
        else:
            self.target_locked = False
            self.latest_fire_info = None

    def control_robot_to_fire(self) -> bool:
        """P-control approach using latest_fire_info"""
        info = self.latest_fire_info
        if not info:
            return False

        dist = float(info.get("dist", 999.0))
        cx = float(info.get("cx", 320.0))

        target_dist = 0.8
        center_x = 320.0

        dist_err = dist - target_dist
        angle_err = center_x - cx

        # arrive?
        if abs(dist_err) < 0.15:
            self.actions.cmd_vel_pub.publish(Twist())
            return True

        cmd = Twist()
        cmd.linear.x = min(0.2 * dist_err, 0.15)
        cmd.angular.z = 0.003 * angle_err

        cmd.linear.x = max(min(cmd.linear.x, 0.2), -0.2)
        cmd.angular.z = max(min(cmd.angular.z, 0.5), -0.5)

        self.actions.cmd_vel_pub.publish(cmd)
        return False

    # ---------------------------------------------------------
    # HELP: callbacks (store + event only)
    # ---------------------------------------------------------
    def _on_help_point(self, msg: Point):
        with self._help_lock:
            self._help_point = (float(msg.x), float(msg.y))
        # self.get_logger().info(f"[HELP] point update {self._help_point}")

    def _on_help_signal(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().warn("[HELP] request received -> preempt flagged")
        self._help_event.set()

    def _handle_help_preempt(self):
        """Stop current work and go to help point (preempt)."""
        with self._help_lock:
            hp = self._help_point

        if hp is None:
            self.get_logger().warn("[HELP] no coordinate yet -> ignore")
            return

        hx, hy = hp
        self.get_logger().warn(f"[HELP] PREEMPT -> go ({hx:.2f}, {hy:.2f})")

        # mark mission running to pause other callbacks logic
        self.is_mission_running = True
        try:
            self.actions.cancel_current_task()
            ok = self.actions.go_to_help_point(hx, hy, timeout=120.0)
            self.get_logger().info(f"[HELP] go_to_help_point ok={ok}")
        finally:
            self.actions.stop_robot()
            self.is_mission_running = False

    # ---------------------------------------------------------
    # Mission loop (single decision point)
    # ---------------------------------------------------------
    def run_mission_sequence(self):
        self.get_logger().info("🧵 run_mission_sequence loop started")

        while rclpy.ok():
            # 0) HELP 최우선
            if self._help_event.is_set():
                self._help_event.clear()
                self._handle_help_preempt()
                time.sleep(0.05)
                continue

            # 1) read code
            with self._code_lock:
                code = self._latest_code

            if not code:
                time.sleep(0.1)
                continue

            now = time.time()

            # debounce
            if code == self._last_handled_code and (now - self._last_handled_time) < self.code_debounce_sec:
                time.sleep(0.05)
                continue

            # already running
            if self.is_mission_running:
                time.sleep(0.1)
                continue

            # dispatch
            if code == "afbfcn":
                self.get_logger().warn("[Mission] code='afbfcn' -> action_1 + fire_search_and_chase")
                self._last_handled_code = code
                self._last_handled_time = now

                self.is_mission_running = True
                try:
                    self.actions.action_1()

                    # chase supports preempt via should_preempt_fn
                    self.actions.fire_search_and_chase(
                        is_target_locked_fn=lambda: self.target_locked,
                        control_to_fire_fn=self.control_robot_to_fire,
                        should_preempt_fn=lambda: self._help_event.is_set(),
                        spin_duration=10.0,
                        chase_timeout=60.0,
                        rotate_speed=0.3,
                    )

                except Exception as e:
                    self.get_logger().error(f"[Mission] failed: {e}")
                finally:
                    self.actions.stop_robot()
                    self.is_mission_running = False
                    self.get_logger().info("[Mission] done")

            elif code == "apbfcn":
                self.get_logger().warn("[Mission] code='apbfcn' -> action_2 (TODO)")
                self._last_handled_code = code
                self._last_handled_time = now

                self.is_mission_running = True
                try:
                    self.actions.action_2()
                except Exception as e:
                    self.get_logger().error(f"[Mission] failed: {e}")
                finally:
                    self.actions.stop_robot()
                    self.is_mission_running = False
                    self.get_logger().info("[Mission] done")

            else:
                self.get_logger().info(f"[Mission] unknown code='{code}' -> ignore")
                self._last_handled_code = code
                self._last_handled_time = now

            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = FullSequenceTest()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.actions.stop_robot()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
