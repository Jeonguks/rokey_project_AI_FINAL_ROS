# fight_fire/action_controller.py
from geometry_msgs.msg import Twist
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from fight_fire.amr_actions import RobotActionLib  # 타입용 import (런타임 영향 없음)


class ActionController:
    """
    Node 아님. TaskControllerNode가 만들어서 함수로 action_1/2/3 호출.
    """
    def __init__(self, node, actions: "RobotActionLib"):
        self.node = node
        self.actions = actions
        self.ns = self.node.get_namespace()

    def _undock_or_fail(self) -> bool:
        if not self.actions.action_undock():
            self.node.get_logger().warn("[Action] undock 실패")
            self.actions.trigger_beep_err()
            return False
        return True

    def _dock_recovery(self) -> bool:
        dock_ok = self.actions.action_dock()
        if dock_ok:
            self.node.get_logger().info("[Action] 도킹 복구 성공")
        else:
            self.node.get_logger().error("[Action] 도킹 복구 실패")
            self.actions.trigger_beep_err()
            try:
                self.actions.cmd_vel_pub.publish(Twist())
            except Exception:
                pass
        return dock_ok

    def action_1(self) -> bool:
        """
        양쪽 방 화재:
          - robot2 -> A 이동 -> 화재 진압
          - robot6 -> B 이동 -> 화재 진압
        """
        self.ns = self.node.get_namespace()
        self.node.get_logger().info(f"[Action1] start (ns={self.ns})")
        self.actions.trigger_beep()

        if not self._undock_or_fail():
            return False

        if self.ns == "/robot2":
            ok = self.actions.go_to_A()
        elif self.ns == "/robot6":
            ok = self.actions.go_to_B()
        else:
            self.node.get_logger().warn(f"[Action1] unknown ns={self.ns}")
            self.actions.trigger_beep_err()
            return False

        if not ok:
            self.node.get_logger().warn("[Action1] 이동 실패 -> 도킹 복구")
            self._dock_recovery()
            return False

        mission_success = self.actions.fire_suppression_mission()
        if mission_success:
            self.actions.trigger_beep_ok()
            self.actions.go_predock()
            self.actions.action_dock()
            return True

        self.actions.trigger_beep_err()
        return False

    def action_2(self) -> bool:
        """
        A방 화재 + B방 사람:
          - robot2 -> A 이동 -> 화재 진압
          - robot6 -> B 이동 -> 인명 안내
        """
        self.ns = self.node.get_namespace()
        self.node.get_logger().info(f"[Action2] start (ns={self.ns})")
        self.actions.trigger_beep()

        if not self._undock_or_fail():
            return False

        if self.ns == "/robot2":
            ok = self.actions.go_to_A()
        elif self.ns == "/robot6":
            ok = self.actions.go_to_B()
        else:
            self.node.get_logger().warn(f"[Action2] unknown ns={self.ns}")
            self.actions.trigger_beep_err()
            return False

        if not ok:
            self.node.get_logger().warn("[Action2] 이동 실패 -> 도킹 복구")
            self._dock_recovery()
            return False

        if self.ns == "/robot2":
            mission_success = self.actions.fire_suppression_mission()
            if mission_success:
                self.actions.trigger_beep_ok()
                self.actions.go_predock()
                self.actions.action_dock()
                return True
            self.actions.trigger_beep_err()
            return False

        # /robot6
        try:
            self.actions.guide_human_sequence()
            self.actions.trigger_beep_ok()
            self.actions.go_predock()
            self.actions.action_dock()
            return True
        except Exception as e:
            self.node.get_logger().error(f"[Action2] guide fail: {e}")
            self.actions.trigger_beep_err()
            return False

    def action_3(self) -> bool:
        """
        B방 화재 + A방 사람:
          - robot6 -> B 이동 -> 화재 진압
          - robot2 -> A 이동 -> 인명 안내
        """
        self.ns = self.node.get_namespace()
        self.node.get_logger().info(f"[Action3] start (ns={self.ns})")
        self.actions.trigger_beep()

        if not self._undock_or_fail():
            return False

        if self.ns == "/robot2":
            ok = self.actions.go_to_A()
        elif self.ns == "/robot6":
            ok = self.actions.go_to_B()
        else:
            self.node.get_logger().warn(f"[Action3] unknown ns={self.ns}")
            self.actions.trigger_beep_err()
            return False

        if not ok:
            self.node.get_logger().warn("[Action3] 이동 실패 -> 도킹 복구")
            self._dock_recovery()
            return False

        if self.ns == "/robot6":
            mission_success = self.actions.fire_suppression_mission()
            if mission_success:
                self.actions.trigger_beep_ok()
                self.actions.go_predock()
                self.actions.action_dock()
                return True
            self.actions.trigger_beep_err()
            return False

        # /robot2
        try:
            self.actions.guide_human_sequence()
            self.actions.trigger_beep_ok()
            self.actions.go_predock()
            self.actions.action_dock()
            return True
        except Exception as e:
            self.node.get_logger().error(f"[Action3] guide fail: {e}")
            self.actions.trigger_beep_err()
            return False
