import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from fight_fire.navigation_client import NavigationClient 
from enum import Enum
import time
import json 

# ------------------------------------------------------------------------------
# [State Machine Definition]
# ------------------------------------------------------------------------------
class MissionState(Enum):
    IDLE = 0                
    UNDOCKING = 1           
    MOVING_TO_WP1 = 11      
    MOVING_TO_WP2 = 2       
    SPINNING = 3            
    CHECK_RESULT = 4        
    EVACUATION_MODE = 5     
    APPROACH_FIRE = 6       
    EXTINGUISHING = 7       
    CALL_FOR_HELP = 8       
    GO_TO_HELP = 9          

# ------------------------------------------------------------------------------
# [MissionCommander Node]
# ------------------------------------------------------------------------------
class MissionCommander(Node):

    def __init__(self):
        super().__init__('mission_commander')
        
        # -- 상태 변수 초기화 --
        self.state = MissionState.IDLE
        self.stand_detected_during_spin = False
        self.extinguish_start_time = 0.0
        
        self.current_x = 0.0
        self.current_y = 0.0

        # -- 내비게이션 클라이언트 --
        self.nav = NavigationClient()
        self.get_logger().info('Mission Commander Ready. Waiting for /webcam_detected (Transient Local)...')

        # ----------------------------------------------------------------------
        # [핵심 수정] QoS 프로필 설정 (상대방 Publisher와 완벽 일치시키기)
        # ----------------------------------------------------------------------
        # 상대방이 'TRANSIENT_LOCAL'(보존성)로 보내고 있다면,
        # 우리도 똑같이 설정해야 늦게 접속해도 데이터를 받아볼 수 있습니다. (등기우편 받기)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,       # 데이터 유실 절대 금지
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 지나간 데이터도 기억했다 받기 (Latching)
            history=HistoryPolicy.KEEP_LAST,              # 마지막 데이터만 기억
            depth=1                                       # 1개만 저장
        )

        # -- Publishers --
        self.fire_mode_pub = self.create_publisher(Bool, 'enable_fire_approach', 10)
        self.evac_pub = self.create_publisher(Bool, 'start_evacuation', 10)
        self.help_pub = self.create_publisher(Point, '/signal/rotation6', 10)

        # -- Subscribers --
        
        # [수정] 위에서 만든 qos_profile을 적용하여 구독
        self.create_subscription(String, '/webcam_detected', self.webcam_callback, qos_profile)

        # 나머지는 기본 QoS(10) 사용
        self.create_subscription(Bool, 'stand_detected', self.stand_callback, 10)
        self.create_subscription(String, 'fire_status', self.fire_status_callback, 10)
        self.create_subscription(Bool, '/signal/rotation2', self.friend_help_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback, 10)
        
        self.timer = self.create_timer(0.5, self.control_loop)

    # --------------------------------------------------------------------------
    # [Event Handlers]
    # --------------------------------------------------------------------------

    def amcl_pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def webcam_callback(self, msg):
        """
        [웹캠 데이터 처리]
        QoS 설정 덕분에 로봇이 늦게 켜져도 이 콜백은 실행됩니다.
        Input: {"class_b_detection": ["fire"], ...}
        """
        
        # 1. 상태 체크
        if self.state != MissionState.IDLE:
            return

        if not msg.data:
            return

        try:
            # 2. JSON 파싱
            data_dict = json.loads(msg.data)
            
            # 3. B카메라 정보 추출
            detected_list = data_dict.get("class_b_detection", [])
            
            # (예외 처리) 문자열로 올 경우 리스트 변환
            if isinstance(detected_list, str):
                detected_list = [detected_list]

        except json.JSONDecodeError:
            self.get_logger().warn(f"JSON 파싱 실패: {msg.data}")
            detected_list = [msg.data]

        # 4. 트리거 매칭
        triggers = ['stand', 'down', 'fire']
        found_triggers = [item for item in detected_list if str(item).lower() in triggers]
        
        if found_triggers:
            self.get_logger().info(f'🚨 [Webcam] B카메라 감지(Latched Msg): {found_triggers}')
            self.get_logger().info('B구역 출동 시퀀스 시작 (Undocking...)')
            
            # [Undock 테스트 로직]
            self.state = MissionState.UNDOCKING
            self.nav.undock()

    def stand_callback(self, msg):
        if self.state == MissionState.SPINNING and msg.data:
            self.stand_detected_during_spin = True

    def fire_status_callback(self, msg):
        if self.state == MissionState.APPROACH_FIRE and msg.data == 'arrived':
            self.get_logger().info('화원 도달. 소화 시퀀스 시작 (30초).')
            self.fire_mode_pub.publish(Bool(data=False)) 
            self.state = MissionState.EXTINGUISHING
            self.extinguish_start_time = time.time()

    def friend_help_callback(self, msg):
        if self.state in [MissionState.IDLE, MissionState.CALL_FOR_HELP]:
            self.get_logger().warn('긴급 지원 요청 수신! Robot2 위치로 이동합니다.')
            self.state = MissionState.GO_TO_HELP
            self.nav.move_to_friend()

    # --------------------------------------------------------------------------
    # [Main Control Loop]
    # --------------------------------------------------------------------------
    def control_loop(self):
        # Step 1: 언독 -> WP1
        if self.state == MissionState.UNDOCKING:
            if self.nav.is_task_complete():
                self.get_logger().info('언독 완료. WP1으로 이동합니다.')
                self.state = MissionState.MOVING_TO_WP1
                self.nav.move_to_wp1()

        # Step 2: WP1 -> WP2
        elif self.state == MissionState.MOVING_TO_WP1:
            if self.nav.is_task_complete():
                if self.nav.is_task_succeeded():
                    self.get_logger().info('WP1 도착. WP2(탕비실)로 이동합니다.')
                    self.state = MissionState.MOVING_TO_WP2
                    self.nav.move_to_wp2()
                else:
                    self.get_logger().error('WP1 이동 실패. 재시도합니다...')
                    self.nav.move_to_wp1()

        # Step 3: WP2 -> Spin
        elif self.state == MissionState.MOVING_TO_WP2:
            if self.nav.is_task_complete():
                if self.nav.is_task_succeeded():
                    self.get_logger().info('B구역 도착. 정찰 회전을 시작합니다.')
                    self.state = MissionState.SPINNING
                    self.stand_detected_during_spin = False
                    self.nav.spin_360()
                else:
                    self.get_logger().error('WP2 이동 실패. 재시도합니다...')
                    self.nav.move_to_wp2()

        # Step 4: Spin -> Check
        elif self.state == MissionState.SPINNING:
            if self.nav.is_task_complete():
                self.state = MissionState.CHECK_RESULT

        # Step 5: Check -> Action
        elif self.state == MissionState.CHECK_RESULT:
            if self.stand_detected_during_spin:
                self.get_logger().info('🛑 인명 발견! 대피 안내 모드로 전환.')
                self.state = MissionState.EVACUATION_MODE
                self.evac_pub.publish(Bool(data=True)) 
            else:
                self.get_logger().info('🔥 인명 없음. 화재 진압 모드로 전환.')
                self.state = MissionState.APPROACH_FIRE
                self.fire_mode_pub.publish(Bool(data=True)) 

        # Step 6: Extinguish
        elif self.state == MissionState.EXTINGUISHING:
            elapsed = time.time() - self.extinguish_start_time
            if elapsed >= 30.0:
                self.get_logger().warn('진압 완료. 본부에 지원 요청 전송.')
                self.state = MissionState.CALL_FOR_HELP
            else:
                if int(elapsed) % 5 == 0:
                    self.get_logger().info(f'소화 진행 중... {int(30-elapsed)}초 남음', throttle_duration_sec=5)

        # Step 7: Call Help
        elif self.state == MissionState.CALL_FOR_HELP:
            help_msg = Point()
            help_msg.x = self.current_x
            help_msg.y = self.current_y
            help_msg.z = 0.0
            self.help_pub.publish(help_msg)
            self.get_logger().info(f'📡 구조 신호 송출 중...', throttle_duration_sec=3)

        # Bonus: Go Help
        elif self.state == MissionState.GO_TO_HELP:
            if self.nav.is_task_complete():
                self.get_logger().info('지원 위치 도착. 대기 상태 전환.')
                self.state = MissionState.IDLE 

def main(args=None):
    rclpy.init(args=args)
    node = MissionCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()