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
        self.ns = self.node.get_namespace()

        self.actions.action_dock()


        # ---------------------------------------------------------
        # 1) Navigator
        # ---------------------------------------------------------
        self.nav = TurtleBot4Navigator()


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
        self._cancel_sent = False
        self._cancel_sent_ts = 0.0

        # 도킹 상태 확인 (예외 방지)
        try:
            if not self.nav.getDockedStatus():
                self.node.get_logger().info('[ActionLib] 도킹 상태가 아닙니다. 도킹을 시도합니다.')
                self.nav.dock()
        except Exception as e:
            self.node.get_logger().warn(f"[ActionLib] Dock status check failed: {e}")

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
            self.node.get_logger().warn(f"Unknown namespace: {self.ns}")

        self.nav.setInitialPose(initial_pose)

        # ❗가능하면 sleep 대신 nav stack ready 확인이 더 좋지만, 일단 유지
        time.sleep(1.0)
        self.node.get_logger().info('[ActionLib] 내비게이션 준비 완료.')


    # =========================================================
    # Actions
    # =========================================================
    
    def action_1(self) -> bool:
        """
        양쪽방 화재 미션.
        return: True(미션 성공) / False(실패 or 중단)
        """
        self.node.get_logger().info(f"[Action1] start (ns={self.ns})")

        self.actions.trigger_beep()

        # 1) 언도킹 (실패하면 종료)
        if not self.actions.action_undock():
            self.node.get_logger().warn("[Action1] undock 실패 -> 미션 중단")
            self.actions.trigger_beep_err()
            return False

        # 2) namespace별 목적지 이동
        if self.ns == "/robot2":
            ok = self.actions.go_to_A()
        elif self.ns == "/robot6":
            ok = self.actions.go_to_B()        
        else:
            self.node.get_logger().warn(f"[Action1] 알 수 없는 ns={self.ns}")
            self.actions.trigger_beep_err()
            return False

        if not ok:
            self.node.get_logger().warn("[Action1] 방 이동 실패 -> 복구 시도: 도킹")
            # 도킹도 실패할 수 있으니 결과 확인
            dock_ok = self.actions.action_dock()
            if dock_ok:
                self.node.get_logger().info("[Action1] 도킹 복구 성공 -> 미션 실패 종료")
            else:
                self.node.get_logger().error("[Action1] 도킹 복구 실패 -> 수동조작 필요(정지/알림)")
                self.actions.trigger_beep_err()
            return False

        # 3) 화재 진압 미션 수행 (탐색 -> 접근 -> 진압 -> 지원요청 -> 복귀)
        self.node.get_logger().info("[Action1] 화재 진압 시퀀스 시작")
        
        # 이 함수 하나가 모든 것을 처리하고, 성공/실패 여부만 반환합니다.
        mission_success = self.actions.fire_suppression_mission()

        if mission_success:
            self.node.get_logger().info("[Action1] 화재 진압 성공! 미션 완료.")
            self.actions.trigger_beep_ok()
            
            #성공 후 복귀
            self.actions.go_predock()
            self.actions.action_dock()
            return True
        else:
            self.node.get_logger().error("[Action1] 화재 진압 실패 (타임아웃 또는 오류)")
            self.actions.trigger_beep_err()
            return False

    def action_2(self) -> bool:
        '''
        A방에 불이 나고 다른 방에 사람이 있는 경우
        미션성공 ->True, 실패->Flase
        '''
        self.node.get_logger().info(f"[Action1] start (ns={self.ns})")
        self.actions.trigger_beep()
        # Undock-> 도킹이 아직 풀리지 않았으면 미션 중단
        if not self.actions.action_undock():
            self.node.get_logger().warn("[Action1] undock 실패 -> 미션 중단")
            self.actions.trigger_beep_err()
            return False
        
        # 2) namespace별 목적지 이동
        if self.ns == "/robot2":
            ok = self.actions.go_to_A()
        elif self.ns == "/robot6":
            ok = self.actions.go_to_B()        
        else:
            self.node.get_logger().warn(f"[Action1] 알 수 없는 ns={self.ns}")
            self.actions.trigger_beep_err()
            return False

        if not ok:
            self.node.get_logger().warn("[Action1] 방 이동 실패 -> 복구 시도: 도킹")
            # 도킹도 실패할 수 있으니 결과 확인
            dock_ok = self.actions.action_dock()
            if dock_ok:
                self.node.get_logger().info("[Action1] 도킹 복구 성공 -> 미션 실패 종료")
            else:
                self.node.get_logger().error("[Action1] 도킹 복구 실패 -> 수동조작 필요(정지/알림)")
                self.actions.trigger_beep_err()
            return False
        
        # 3) 화재 진압 미션 수행 (탐색 -> 접근 -> 진압 -> 지원요청 -> 복귀)
        if self.ns == "/robot2":
            self.node.get_logger().info("[Action1] A 화재 진압 시퀀스 시작")
            mission_success = self.actions.fire_suppression_mission()

            if mission_success:
                self.node.get_logger().info("[Action1] 화재 진압 성공! 미션 완료.")
                self.actions.trigger_beep_ok()
                
                #성공 후 복귀
                self.actions.go_predock()
                self.actions.action_dock()
                return True
            else:
                self.node.get_logger().error("[Action1] 화재 진압 실패 (타임아웃 또는 오류)")
                self.actions.trigger_beep_err()
                return False
            
        elif self.ns == "/robot6":
            self.node.get_logger().info("[Action2] B 인명 구조(에스코트) 시퀀스 시작")
    
            try:
                self.actions.guide_human_sequence()
                
                # 안내가 정상적으로 끝나면 여기로 옴
                self.node.get_logger().info("[Action2] 인명 구조 완료! 복귀합니다.")
                self.actions.trigger_beep_ok()
                
                # 복귀 및 도킹
                self.actions.go_predock()
                self.actions.action_dock()
                return True

            except Exception as e:
                # 도중에 에러나면 실패 처리
                self.node.get_logger().error(f"[Action2] 구조 실패: {e}")
                self.actions.trigger_beep_err()
                return False

    def action_3(self) -> bool:
        '''
        B방에 불이 나고 A방에 사람이 있는 경우
        미션성공 -> True, 실패 -> False
        '''
        self.node.get_logger().info(f"[Action2] start (ns={self.ns})")
        self.actions.trigger_beep()

        # Undock-> 도킹이 아직 풀리지 않았으면 미션 중단
        if not self.actions.action_undock():
            self.node.get_logger().warn("[Action2] undock 실패 -> 미션 중단")
            self.actions.trigger_beep_err()
            return False

        # 2) namespace별 목적지 이동 (뒤집기)
        # - robot2: A방(사람)으로
        # - robot6: B방(불)로
        if self.ns == "/robot2":
            ok = self.actions.go_to_A()   # 사람 있는 A
        elif self.ns == "/robot6":
            ok = self.actions.go_to_B()   # 불 난 B
        else:
            self.node.get_logger().warn(f"[Action2] 알 수 없는 ns={self.ns}")
            self.actions.trigger_beep_err()
            return False

        if not ok:
            self.node.get_logger().warn("[Action2] 방 이동 실패 -> 복구 시도: 도킹")
            dock_ok = self.actions.action_dock()
            if dock_ok:
                self.node.get_logger().info("[Action2] 도킹 복구 성공 -> 미션 실패 종료")
            else:
                self.node.get_logger().error("[Action2] 도킹 복구 실패 -> 수동조작 필요(정지/알림)")
                self.actions.trigger_beep_err()
            return False

        # 3) 시퀀스 수행 (뒤집기)
        # - robot6: B 화재 진압
        # - robot2: A 인명 구조/에스코트
        if self.ns == "/robot6":
            self.node.get_logger().info("[Action2] B 화재 진압 시퀀스 시작")
            mission_success = self.actions.fire_suppression_mission()

            if mission_success:
                self.node.get_logger().info("[Action2] 화재 진압 성공! 미션 완료.")
                self.actions.trigger_beep_ok()

                # 성공 후 복귀
                self.actions.go_predock()
                self.actions.action_dock()
                return True
            else:
                self.node.get_logger().error("[Action2] 화재 진압 실패 (타임아웃 또는 오류)")
                self.actions.trigger_beep_err()
                return False

        elif self.ns == "/robot2":
            self.node.get_logger().info("[Action2] A 인명 구조(에스코트) 시퀀스 시작")
            try:
                self.actions.guide_human_sequence()

                self.node.get_logger().info("[Action2] 인명 구조 완료! 복귀합니다.")
                self.actions.trigger_beep_ok()

                # 복귀 및 도킹
                self.actions.go_predock()
                self.actions.action_dock()
                return True

            except Exception as e:
                self.node.get_logger().error(f"[Action2] 구조 실패: {e}")
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