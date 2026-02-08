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

    # =========================================================
    # Actions
    # =========================================================
    
    def action_1(self):
        """
        양쪽방에 불났음 
        FullSequenceTest에서 code == 'afbfc?'일 때 호출.
        ns에 따라 robot2/robot6 이동 루트를 분기.
        """
        ns = self.node.get_namespace()
        self.node.get_logger().info(f"[Action1] start (ns={ns})")

        self.trigger_beep()
        self.actions.action_undock()

        # Namespace별 경로
        if ns == "/robot2":
            # robot2 -> 장소 A 루트 (a1->a2->a3)
            self.actions.move_to_wp_a1(); self.wait_for_nav(step_name="wp_a1")
            self.actions.move_to_wp_a2(); self.wait_for_nav(step_name="wp_a2")
            self.actions.move_to_wp_a3(); self.wait_for_nav(step_name="wp_a3")
            

        elif ns == "/robot6":
            # robot6 -> 장소 B 루트 (b1->b2)
            self.actions.move_to_wp_b1(); self.wait_for_nav(step_name="wp_b1")
            self.actions.move_to_wp_b2(); self.wait_for_nav(step_name="wp_b2")

        else:
            self.node.get_logger().warn(
                f"[Action1] unknown namespace: {ns}. "
                "실행 시 --ros-args -r __ns:=/robot2 또는 /robot6 로 지정했는지 확인"
            )
            return

        self.node.get_logger().info("[Action1] done")


    def action_2(self):
        pass

        # 방안에 불도 있고 사람도 있을떄 로봇 
        # 로봇 둘중 하나는 불끄고 나머지 하나는 사람 구해야함 
        # 가까운 로봇 하나가 불 난곳 이동.
        # 나머지 로봇 하나가 불 난곳 이동.

        
    ################################################
        # 이건 태스크 매니저에서 따로 추가 하자 ? 몰라 무슨의도로 썼는지 까먹음 



        # 
        # 가까운 로봇은 가서 불끄기
        # 사람은 



    def action_3(self):
        '''
        A방에 불이 나서 A로봇이 불 끄러가고, B로봇이 B방에 사람 있는지 보러 가는 시나리오
        30초후 B에게 도움 요청
        '''
        # Undock
        self.action_undock()
        # 도킹이 아직 풀리지 않았으면 미션 중단(안전)
        if self.nav.getDockedStatus():
            self.node.get_logger().error("[Action3] undock 실패 -> 중단")
            return
        
        # 2) Namespace별 경로
        if self.namespace == "/robot2":
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

            else:
                self.node.get_logger().warn("❌ 화재를 찾지 못해 다음방으로 감.")
                self.move_to_wp_a3(); self.wait_for_nav(step_name="wp_a3")
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
                    self.action_undock()  

        elif self.namespace == "/robot6":
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