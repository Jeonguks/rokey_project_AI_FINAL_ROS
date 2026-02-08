import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
import math
from fight_fire.amr_actions import RobotActionLib


class DoActionNode(Node):
    def __init__(self):
        super().__init__('do_action_node')

        # ActionLib 연결
        self.actions = RobotActionLib(self)

        # 현재 namespace (/robot2, /robot6)
        self.ns = self.get_namespace()

        # 초기 도킹 시도
        self.actions.action_dock()

        # Navigator 생성
        self.nav = TurtleBot4Navigator()

        # robot2 초기 위치
        self.initial_pose_robot2 = {
            "x": 3.710208,
            "y": 2.242908,
            "qz": 0.6096358,
            "qw": 0.7926816,
        }

        # robot6 초기 위치
        self.initial_pose_robot6 = {
            "x": -0.117279,
            "y": 0.019432,
            "qz": -0.771297,
            "qw": 0.636474,
        }

        # robot2 predock 위치
        self.predock_pose_robot2 = {
            "x": 3.70,
            "y": 2.00,
            "qz": 0.6096358,
            "qw": 0.7926816,
        }

        # robot6 predock 위치
        self.predock_pose_robot6 = {
            "x": -0.00918,
            "y": 0.004354,
            "qz": -0.771297,
            "qw": 0.636474,
        }

        # cancel 플래그
        self._cancel_sent = False
        self._cancel_sent_ts = 0.0

        # 현재 도킹 상태 확인 후 도킹 시도
        try:
            if not self.nav.getDockedStatus():
                self.get_logger().info('[ActionLib] 도킹 상태가 아닙니다. 도킹을 시도합니다.')
                self.nav.dock()
        except Exception as e:
            self.get_logger().warn(f"[ActionLib] Dock status check failed: {e}")

        # AMCL 초기 위치 설정
        if self.ns == "/robot2":
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.initial_pose_robot2["x"]
            initial_pose.pose.position.y = self.initial_pose_robot2["y"]
            initial_pose.pose.orientation.z = self.initial_pose_robot2["qz"]
            initial_pose.pose.orientation.w = self.initial_pose_robot2["qw"]

        elif self.ns == "/robot6":
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.initial_pose_robot6["x"]
            initial_pose.pose.position.y = self.initial_pose_robot6["y"]
            initial_pose.pose.orientation.z = self.initial_pose_robot6["qz"]
            initial_pose.pose.orientation.w = self.initial_pose_robot6["qw"]

        else:
            self.get_logger().warn(f"Unknown namespace: {self.ns}")

        # Nav2 초기 pose 전달
        self.nav.setInitialPose(initial_pose)

        # Nav stack 준비 대기
        time.sleep(1.0)
        self.get_logger().info('[ActionLib] 내비게이션 준비 완료.')

    # =========================================================
    # Actions
    # =========================================================

    def action_1(self) -> bool:
        """
        양쪽 방 화재 미션
        """
        self.get_logger().info(f"[Action1] start (ns={self.ns})")

        # 시작 비프
        self.actions.trigger_beep()

        # 언도킹
        if not self.actions.action_undock():
            self.get_logger().warn("[Action1] undock 실패 -> 미션 중단")
            self.actions.trigger_beep_err()
            return False

        # 목적지 이동
        if self.ns == "/robot2":
            ok = self.actions.go_to_A()
        elif self.ns == "/robot6":
            ok = self.actions.go_to_B()
        else:
            self.get_logger().warn(f"[Action1] 알 수 없는 ns={self.ns}")
            self.actions.trigger_beep_err()
            return False

        # 이동 실패 시 도킹 복구
        if not ok:
            self.get_logger().warn("[Action1] 방 이동 실패 -> 도킹 복구 시도")
            dock_ok = self.actions.action_dock()
            if dock_ok:
                self.get_logger().info("[Action1] 도킹 복구 성공")
            else:
                self.get_logger().error("[Action1] 도킹 복구 실패")
                self.actions.trigger_beep_err()
            return False

        # 화재 진압 시퀀스
        self.get_logger().info("[Action1] 화재 진압 시퀀스 시작")
        mission_success = self.actions.fire_suppression_mission()

        if mission_success:
            self.get_logger().info("[Action1] 화재 진압 성공")
            self.actions.trigger_beep_ok()
            self.actions.go_predock()
            self.actions.action_dock()
            return True
        else:
            self.get_logger().error("[Action1] 화재 진압 실패")
            self.actions.trigger_beep_err()
            return False

    def action_2(self) -> bool:
        """
        A방 화재 + B방 인명 존재
        """
        self.get_logger().info(f"[Action2] start (ns={self.ns})")
        self.actions.trigger_beep()

        # 언도킹
        if not self.actions.action_undock():
            self.get_logger().warn("[Action2] undock 실패")
            self.actions.trigger_beep_err()
            return False

        # 이동
        if self.ns == "/robot2":
            ok = self.actions.go_to_A()
        elif self.ns == "/robot6":
            ok = self.actions.go_to_B()
        else:
            self.get_logger().warn(f"[Action2] 알 수 없는 ns={self.ns}")
            self.actions.trigger_beep_err()
            return False

        # 이동 실패 시 도킹
        if not ok:
            self.get_logger().warn("[Action2] 이동 실패 -> 도킹 시도")
            dock_ok = self.actions.action_dock()
            if not dock_ok:
                self.get_logger().error("[Action2] 도킹 실패")
                self.actions.trigger_beep_err()
            return False

        # robot2: 화재 진압
        if self.ns == "/robot2":
            self.get_logger().info("[Action2] A 화재 진압 시작")
            mission_success = self.actions.fire_suppression_mission()

            if mission_success:
                self.actions.trigger_beep_ok()
                self.actions.go_predock()
                self.actions.action_dock()
                return True
            else:
                self.actions.trigger_beep_err()
                return False

        # robot6: 인명 구조
        elif self.ns == "/robot6":
            self.get_logger().info("[Action2] B 인명 구조 시작")
            try:
                self.actions.guide_human_sequence()
                self.actions.trigger_beep_ok()
                self.actions.go_predock()
                self.actions.action_dock()
                return True
            except Exception as e:
                self.get_logger().error(f"[Action2] 구조 실패: {e}")
                self.actions.trigger_beep_err()
                return False

    def action_3(self) -> bool:
        """
        B방 화재 + A방 인명 존재
        """
        self.get_logger().info(f"[Action3] start (ns={self.ns})")
        self.actions.trigger_beep()

        # 언도킹
        if not self.actions.action_undock():
            self.get_logger().warn("[Action3] undock 실패")
            self.actions.trigger_beep_err()
            return False

        # 이동
        if self.ns == "/robot2":
            ok = self.actions.go_to_A()
        elif self.ns == "/robot6":
            ok = self.actions.go_to_B()
        else:
            self.get_logger().warn(f"[Action3] 알 수 없는 ns={self.ns}")
            self.actions.trigger_beep_err()
            return False

        # 이동 실패 시 도킹
        if not ok:
            self.get_logger().warn("[Action3] 이동 실패 -> 도킹 시도")
            dock_ok = self.actions.action_dock()
            if not dock_ok:
                self.get_logger().error("[Action3] 도킹 실패")
                self.actions.trigger_beep_err()
            return False

        # robot6: 화재 진압
        if self.ns == "/robot6":
            self.get_logger().info("[Action3] B 화재 진압 시작")
            mission_success = self.actions.fire_suppression_mission()

            if mission_success:
                self.actions.trigger_beep_ok()
                self.actions.go_predock()
                self.actions.action_dock()
                return True
            else:
                self.actions.trigger_beep_err()
                return False

        # robot2: 인명 구조
        elif self.ns == "/robot2":
            self.get_logger().info("[Action3] A 인명 구조 시작")
            try:
                self.actions.guide_human_sequence()
                self.actions.trigger_beep_ok()
                self.actions.go_predock()
                self.actions.action_dock()
                return True
            except Exception as e:
                self.get_logger().error(f"[Action3] 구조 실패: {e}")
                self.actions.trigger_beep_err()
                return False


def main(args=None):
    rclpy.init(args=args)
    node = DoActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()