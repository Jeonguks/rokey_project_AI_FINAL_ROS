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
        # Undock
        self.action_undock()
        # 도킹이 아직 풀리지 않았으면 미션 중단(안전)
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
            self.node.get_logger().info("[Action1] B 인명 구조 시퀀스 시작")
            mission_success = self.actions.fire_suppression_mission()



    def action_3(self) -> bool:
        '''

        '''
        pass




    def action_4(self):
        '''
        B방에 불이 나서 B로봇이 불 끄러가고, A로봇이 A방에 사람 있는지 보러 가는 시나리오
        30초후 A에게 도움 요청
        '''
        # Undock
        self.action_undock()
        # 도킹이 아직 풀리지 않았으면 미션 중단(안전)
        if self.nav.getDockedStatus():
            self.node.get_logger().error("[Action3] undock 실패 -> 중단")
            return
        
        # 2) Namespace별 경로
        if self.namespace == "/robot6":
            # robot2 -> 장소 A 루트 (a1->a2->a3)
            self.node.get_logger().info("Starting Mission.")
            self.move_to_wp_a1(); self.wait_for_nav(step_name="wp_a1")
            self.move_to_wp_a2(); self.wait_for_nav(step_name="wp_a2")

            found = self.spin_and_search_fire(timeout=15.0)  
            if found:
                self.moving_pub.publish(String(data="화재 접근 중"))
                # 찾은 상태에서 그대로 접근 시작
                self.action_approach_fire()
                # 30초 경과후 도움요청
                self.send_help_point(self.robot_x, self.robot_y)
                # 도움요청후 프리도킹 위치로 이동
                self.go_predock()
                # 도킹
                self.action_dock()

        elif self.namespace == "/robot2":
            # robot6 -> 장소 B 루트 (b1->b2)
            self.move_to_wp_b1(); self.wait_for_nav(step_name="wp_b1")
            if self._handle_help_interrupt_robot6(step_name="after_wp_b1"):
                return
            self.move_to_wp_b2(); self.wait_for_nav(step_name="wp_b2")
            if self._handle_help_interrupt_robot6(step_name="after_wp_b2"):
                return
            self.node.get_logger().info("[Action3] robot6: help 대기 모드 진입")

            while rclpy.ok():
                # ✅ 도움 들어오면 즉시 이동
                if self._handle_help_interrupt_robot6(step_name="wait_loop"):
                    return

                # 대기 중에도 안전하게 정지 유지
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.2)
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


        #

        ###############################################3
    def action_14(self):
        # 언제든 상관없이 누워있는 사람 (Down)발견 하면 다른 로봇에게 좌표 전송
        #좌표전송은 pub _target_point파일 참고
        pass

    def action_5(self):
        # 시작은 대피중인 로봇이 도움핑 받으면 무조건 사람부터 대기 시킴 
        # 진화작업 30초 지나면 다른로봇에게 도움 요청하기. 
        # 좌표전송
        # 30초 지난 로봇은 잠시 비켜있기
        # 도움 핑 받은 로봇은 그곳으로 가기
        # 도움 핑 받은 로봇은 불 끄기 
        # 나와있는로봇은 순회모드, 
        # 충돌 피할 수 있으면 교차 하든가 ㅋㅋ 
        pass


    def action_6(self):
        # 불끄고 30초 지나면 도와주러 오는지 테스트

        found = self.spin_and_search_fire(timeout=15.0)  
        if found:
            self.moving_pub.publish(String(data="화재 접근 중"))
            # 찾은 상태에서 그대로 접근 시작
            self.action_approach_fire()
            # 30초 경과후 도움요청
            self.send_help_point()
            # 도움요청후 프리도킹 위치로 이동
            self.go_predock()
            # 도킹
            self.action_dock()

        else:
            self.node.get_logger().warn("❌ 화재를 찾지 못해 접근 단계를 건너뜁니다.")    


    def action_7(self):
        if self.namespace == "/robot2":
            # robot2 -> 장소 A 루트 (a1->a2->a3)
            self.node.get_logger().info("Starting Mission.")

            

        elif self.namespace == "/robot6":
            # robot6 -> 장소 B 루트 (b1->b2)
            self.move_to_wp_b1(); self.wait_for_nav(step_name="wp_b1")
            self.move_to_wp_b2(); self.wait_for_nav(step_name="wp_b2")       

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