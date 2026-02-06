import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
import math

class EvacuationState:
    IDLE = 0                
    MOVING_TO_SHELTER = 1   
    TURNING_BACK = 2        
    WAITING_FOR_HUMAN = 3   
    RESUMING_PATH = 4       
    ARRIVED_TURNING = 5     # [NEW] 도착 후 뒤로 도는 상태
    ARRIVED_ALIGNING = 6    # [NEW] 사람을 보고 시선 고정하는 상태
    COMPLETE = 7            

class EvacuationNode(Node):
    def __init__(self):
        super().__init__('evacuation_node')
        self.namespace = 'robot6'
        
        self.state = EvacuationState.IDLE
        
        self.check_interval = 5.0 
        self.safe_distance = 1.0   
        
        # [수정 1] 회전 튜닝 (조금 천천히, 더 오래 돌기)
        self.turn_speed = 0.8      # 속도를 살짝 줄여서 안정적으로
        self.turn_duration = 5.0   # 시간을 늘려서 180도+ 확보 (0.8 * 5 = 4.0rad ≈ 229도)
        
        self.last_check_time = 0.0 
        self.turn_start_time = 0.0 
        self.last_seen_time = 0.0  
        self.debug_timer = 0.0     

        self.latest_stand_info = [0.0, 0.0, 0.0] 
        self.goal_handle = None

        self.navigator = TurtleBot4Navigator(namespace=self.namespace)
        
        # Action Client 설정
        action_topic = f'/{self.namespace}/navigate_to_pose'
        self.nav_client = ActionClient(self, NavigateToPose, action_topic)

        if not self.navigator.getDockedStatus():
            self.get_logger().info(f'안내견 준비 완료. Action Server: {action_topic}')

        # 토픽 설정
        self.create_subscription(Bool, '/start_evacuation', self.evac_start_callback, 10)
        self.create_subscription(Float32MultiArray, '/stand_info', self.stand_info_callback, 10)
        
        self.evac_status_pub = self.create_publisher(String, '/evacuation_status', 10)
        self.perception_switch_pub = self.create_publisher(Bool, '/enable_person_detection', 10)

        # [중요] 로봇 회전 명령 토픽
        cmd_vel_topic = f'/{self.namespace}/cmd_vel'
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.create_timer(0.1, self.control_loop)

    def get_current_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def stand_info_callback(self, msg):
        self.latest_stand_info = msg.data
        if msg.data[2] == 1.0: 
            self.last_seen_time = self.get_current_time_sec()

    def evac_start_callback(self, msg):
        if msg.data and self.state == EvacuationState.IDLE:
            self.get_logger().warn("🚨 대피 시퀀스 시작! 대피소로 이동합니다.")
            self.state = EvacuationState.MOVING_TO_SHELTER
            
            now = self.get_current_time_sec()
            self.last_check_time = now
            self.debug_timer = now 
            
            self.perception_switch_pub.publish(Bool(data=False))
            self.start_navigation() 

    def start_navigation(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = 0.972021
        goal_msg.pose.pose.position.y = -0.383458
        goal_msg.pose.pose.orientation.w = 1.0 

        self.get_logger().info("👉 이동 명령 전송 (Async)... Nav2 연결 대기...")
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 Action Server 연결 실패! 네임스페이스 확인 필요.")
            return

        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 이동 명령 거부됨.')
            return
        self.goal_handle = goal_handle 
        self.get_logger().info('✅ 이동 명령 승인됨. 주행 시작.')

    def stop_robot(self):
        if self.goal_handle:
            self.get_logger().info("🛑 Nav2 이동 취소 요청...")
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

    def rotate_robot(self, speed):
        cmd = Twist()
        cmd.angular.z = float(speed)
        self.cmd_pub.publish(cmd)

    def get_distance_to_goal(self):
        try:
            current_pose = self.navigator.getPose()
            target_x = 0.972021
            target_y = -0.383458
            dx = target_x - current_pose.pose.position.x
            dy = target_y - current_pose.pose.position.y
            return math.sqrt(dx**2 + dy**2)
        except:
            return 999.9

    def control_loop(self):
        current_time = self.get_current_time_sec()

        # State 1: 이동 중
        if self.state == EvacuationState.MOVING_TO_SHELTER:
            if current_time - self.debug_timer > 1.0:
                elapsed = current_time - self.last_check_time
                self.get_logger().info(f"⏳ 이동 중... 다음 확인까지 {5.0 - elapsed:.1f}초")
                self.debug_timer = current_time

            dist = self.get_distance_to_goal()
            if dist < 0.15: 
                self.get_logger().info("✅ 대피소 도착! (거리 < 0.15m)")
                self.stop_robot()
                
                # [수정] 도착 후 바로 끝내지 않고, 뒤로 돌기 상태로 전환
                self.state = EvacuationState.ARRIVED_TURNING
                self.turn_start_time = current_time
                return

            if current_time - self.last_check_time > self.check_interval:
                self.get_logger().info("👀 5초 경과! 이동을 멈추고 뒤를 확인합니다.")
                self.stop_robot()
                self.state = EvacuationState.TURNING_BACK
                self.turn_start_time = current_time

        # State 2: 중간 점검 - 뒤로 돌기
        elif self.state == EvacuationState.TURNING_BACK:
            if current_time - self.turn_start_time < self.turn_duration: 
                self.rotate_robot(self.turn_speed) 
            else:
                self.stop_robot()
                self.state = EvacuationState.WAITING_FOR_HUMAN
                self.get_logger().info("🛑 회전 완료. 사람 탐지(ON) 시작.")
                self.perception_switch_pub.publish(Bool(data=True))

        # State 3: 중간 점검 - 사람 기다리기
        elif self.state == EvacuationState.WAITING_FOR_HUMAN:
            dist = self.latest_stand_info[0]
            detected = (self.latest_stand_info[2] == 1.0)
            
            if (detected and dist <= self.safe_distance):
                self.get_logger().info(f"👌 거리 확보됨 ({dist:.2f}m). 탐지 끄고 이동 재개.")
                self.perception_switch_pub.publish(Bool(data=False))
                self.state = EvacuationState.RESUMING_PATH
                self.turn_start_time = current_time
            else:
                # 감지되면 화면 중앙 맞추기
                center_x = self.latest_stand_info[1]
                if detected:
                    err = 320 - center_x 
                    k = 0.003 
                    self.rotate_robot(k * err)

        # State 4: 다시 앞 보기 & 이동 재개
        elif self.state == EvacuationState.RESUMING_PATH:
            if current_time - self.turn_start_time < self.turn_duration:
                self.rotate_robot(-self.turn_speed) 
            else:
                self.stop_robot()
                self.state = EvacuationState.MOVING_TO_SHELTER
                self.start_navigation() 
                self.last_check_time = current_time 
                self.debug_timer = current_time

        # State 5: [NEW] 도착 후 뒤로 돌기
        elif self.state == EvacuationState.ARRIVED_TURNING:
            if current_time - self.turn_start_time < self.turn_duration:
                self.rotate_robot(self.turn_speed) # 180도 회전
            else:
                self.stop_robot()
                self.get_logger().info("🏁 최종 회전 완료. 사람을 찾으며 시선을 고정합니다.")
                self.state = EvacuationState.ARRIVED_ALIGNING
                self.perception_switch_pub.publish(Bool(data=True)) # 탐지 ON

        # State 6: [NEW] 도착 후 시선 고정 (Visual Servoing)
        elif self.state == EvacuationState.ARRIVED_ALIGNING:
            detected = (self.latest_stand_info[2] == 1.0)
            
            if detected:
                # [핵심] 바운딩 박스 중앙 맞추기 로직
                center_x = self.latest_stand_info[1]
                err = 320 - center_x
                k = 0.004 # 반응 속도 계수
                
                cmd = Twist()
                cmd.angular.z = float(k * err)
                # 속도 제한 (너무 빠르지 않게)
                cmd.angular.z = max(min(cmd.angular.z, 0.5), -0.5)
                self.cmd_pub.publish(cmd)
                
                # 로그를 너무 자주 찍지 않게 조절
                if current_time - self.debug_timer > 2.0:
                    self.get_logger().info(f"👀 사람 추적 중... (오차: {err:.1f})")
                    self.debug_timer = current_time
            else:
                # 사람이 안 보이면? 천천히 제자리 회전하며 찾기
                self.rotate_robot(0.3)
                if current_time - self.debug_timer > 2.0:
                    self.get_logger().info("🔎 사람 찾는 중...")
                    self.debug_timer = current_time

def main(args=None):
    rclpy.init(args=args)
    node = EvacuationNode()
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