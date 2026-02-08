# fight_fire/task_controller_node.py
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

from fight_fire.action_controller import ActionController
from fight_fire.amr_actions import RobotActionLib


class TaskControllerNode(Node):
    def __init__(self):
        super().__init__("task_controller_node")

        # ns (/robot2 or /robot6)
        self.namespace = self.get_namespace()
        self.other_namespace = "/robot6" if self.namespace == "/robot2" else "/robot2"

        # low-level actions + controller
        self.actions = RobotActionLib(self)
        self.controller = ActionController(self, self.actions)

        # ---- trigger/detection inputs ----
        self.create_subscription(String, "/webcam_detected", self.trigger_callback, 10)
        self.create_subscription(String, "perception/detections", self.perception_callback, 10)

        # ---- help interrupt ----
        self.create_subscription(Bool, f"{self.other_namespace}/signal/help", self.get_help_trigger, 10)

        # mission state
        self.is_mission_running = False
        self.target_locked = False
        self.latest_fire_info = None

        # latest code cache
        self._code_lock = threading.Lock()
        self._latest_code = None
        self._latest_code_time = 0.0

        # debounce
        self._last_handled_code = None
        self._last_handled_time = 0.0
        self.code_debounce_sec = 2.0

        # help cache
        self._help_lock = threading.Lock()
        self._help_requested = False
        self._help_goal = None  # (x,y)

        # mission loop thread
        self._mission_thread = threading.Thread(target=self.run_mission_sequence, daemon=True)
        self._mission_thread.start()

        self.get_logger().info("TaskControllerNode ready")

    # ---------------------------
    # code build helpers
    # ---------------------------
    def map_detection_to_code(self, detection_list):
        if not detection_list:
            return "n"
        s = set(detection_list)
        if "fire" in s:
            return "f"
        if "stand" in s:
            return "s"
        if "down" in s:
            return "d"
        return "n"

    def build_code_from_detection(self, data: dict) -> str:
        a_code = self.map_detection_to_code(data.get("class_a_detection", []))
        b_code = self.map_detection_to_code(data.get("class_b_detection", []))
        c_code = self.map_detection_to_code(data.get("class_c_detection", []))
        return f"a{a_code}b{b_code}c{c_code}"

    # ---------------------------
    # callbacks
    # ---------------------------
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

        self.get_logger().info(f"[Trigger RX] code='{code}'")

    def perception_callback(self, msg: String):
        # ActionLib already subscribes this too; here we only maintain TaskController flags if needed
        if not self.is_mission_running:
            return
        try:
            data = json.loads(msg.data)
        except Exception:
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

    # ---------------------------
    # help interrupt
    # ---------------------------
    def get_help_trigger(self, msg: Bool):
        if not msg.data:
            return
        if self.actions.last_point is None:
            self.get_logger().warn("[HELP] help=True but no coordinate yet")
            return

        with self._help_lock:
            self._help_requested = True
            self._help_goal = self.actions.last_point

        self.get_logger().warn(f"[HELP] interrupt requested -> {self._help_goal}")

    def _consume_help_request(self):
        with self._help_lock:
            if (not self._help_requested) or (self._help_goal is None):
                return None
            goal = self._help_goal
            self._help_requested = False
            self._help_goal = None
            return goal

    def _interrupt_if_help(self) -> bool:
        goal = self._consume_help_request()
        if goal is None:
            return False

        hx, hy = goal
        self.get_logger().warn(f"[HELP] INTERRUPT -> go ({hx:.3f},{hy:.3f})")

        # 1) cancel current nav
        try:
            self.actions.nav.cancelTask()
        except Exception:
            pass

        # 2) stop cmd_vel
        try:
            self.actions.cmd_vel_pub.publish(Twist())
        except Exception:
            pass

        # 3) go help goal
        goal_pose = self.actions.nav.getPoseStamped([hx, hy], self.actions.NAV_DIR_HELP)
        self.actions.nav.startToPose(goal_pose)
        self.actions.wait_for_nav(timeout=120.0, step_name="help_goal")

        return True

    # ---------------------------
    # mission loop
    # ---------------------------
    def run_mission_sequence(self):
        self.get_logger().info("run_mission_sequence started")
        while rclpy.ok():
            # help is priority
            if self._interrupt_if_help():
                self.is_mission_running = False
                self.target_locked = False
                self.latest_fire_info = None

            with self._code_lock:
                code = self._latest_code
                _ = self._latest_code_time

            if not code:
                time.sleep(0.1)
                continue

            now = time.time()
            if code == self._last_handled_code and (now - self._last_handled_time) < self.code_debounce_sec:
                time.sleep(0.05)
                continue

            if self.is_mission_running:
                time.sleep(0.1)
                continue

            # -------- code routing --------
            if code == "afbfcn":
                # 양쪽방 불
                self._last_handled_code = code
                self._last_handled_time = now
                self.is_mission_running = True
                try:
                    self.controller.action_1()
                except Exception as e:
                    self.get_logger().error(f"[Mission] action_1 failed: {e}")
                finally:
                    self.actions.stop_robot()
                    self.is_mission_running = False

            elif code == "afbscn":
                # A불 + B사람
                self._last_handled_code = code
                self._last_handled_time = now
                self.is_mission_running = True
                try:
                    self.controller.action_2()
                except Exception as e:
                    self.get_logger().error(f"[Mission] action_2 failed: {e}")
                finally:
                    self.actions.stop_robot()
                    self.is_mission_running = False

            elif code == "asbfcn":
                # B불 + A사람
                self._last_handled_code = code
                self._last_handled_time = now
                self.is_mission_running = True
                try:
                    self.controller.action_3()
                except Exception as e:
                    self.get_logger().error(f"[Mission] action_3 failed: {e}")
                finally:
                    self.actions.stop_robot()
                    self.is_mission_running = False

            else:
                self._last_handled_code = code
                self._last_handled_time = now

            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = TaskControllerNode()
    executor = MultiThreadedExecutor()
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
