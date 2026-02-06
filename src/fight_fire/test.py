import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import threading
import time
import json
from fight_fire.amr_actions import RobotActionLib

class FullSequenceTest(Node):
    def __init__(self):
        super().__init__('full_sequence_test_node')
        
        # ActionLib 연결
        self.actions = RobotActionLib(self)
        
        # 1. 트리거 구독
        self.create_subscription(
            String, 
            '/webcam_detected', 
            self.trigger_callback, 
            10
        )

        # 2. Perception의 JSON 데이터 구독
        self.create_subscription(
            String,
            'perception/detections',
            self.perception_callback,
            10
        )
        
        # 상태 변수
        self.is_mission_running = False # 미션 수행중인가 ?  
        self.target_locked = False   # 불을 발견했는지?
        self.latest_fire_info = None # 불의 위치 정보

        # -------------------------
        # /webcam_detected 코드 저장소
        # -------------------------
        self._code_lock = threading.Lock()
        self._latest_code = None
        self._latest_code_time = 0.0

        # 중복 처리 방지 (디바운스)
        self._last_handled_code = None
        self._last_handled_time = 0.0
        self.code_debounce_sec = 2.0   # 같은 코드 반복 들어올 때 무시 시간

        self.get_logger().info("✅ 테스트 노드 준비 완료!")
        self.get_logger().info("'/webcam_detected' 토픽을 기다리는 중...")
        # -------------------------
        # 미션 루프 스레드 시작 (항상 돌면서 코드 조건 검사)
        # -------------------------
        self._mission_thread = threading.Thread(
            target=self.run_mission_sequence,
            daemon=True
        )
        self._mission_thread.start()


    # ---------------------------------------------------------
    # (A) /webcam_detected: 콜백은 코드 저장만!
    # ---------------------------------------------------------
    def trigger_callback(self, msg: String):
        code = (msg.data or "").strip()
        with self._code_lock:
            self._latest_code = code
            self._latest_code_time = time.time()

        self.get_logger().info(f"[Trigger RX] code='{code}'")

    def perception_callback(self, msg):
        """Perception Node가 보내주는 JSON 데이터를 해석"""
        if not self.is_mission_running: return

        try:
            data = json.loads(msg.data)
            fire_obj = None
            
            # JSON 리스트에서 'fire' 클래스 찾기
            for obj in data:
                if obj['class'] == 'fire':
                    fire_obj = obj
                    break
            
            if fire_obj:
                self.target_locked = True
                self.latest_fire_info = fire_obj
            else:
                self.target_locked = False
                
        except json.JSONDecodeError:
            pass

    def control_robot_to_fire(self):
        """JSON 데이터를 기반으로 로봇을 제어하는 함수 (P-Control)"""
        if not self.latest_fire_info: return False 

        # 정보 추출
        dist = self.latest_fire_info['dist'] # 거리 (m)
        cx = self.latest_fire_info['cx']     # 화면 중심 좌표 (x)
        
        target_dist = 0.8  # 목표 거리 (m)
        center_x = 320     # 이미지 가로 640의 절반

        # 오차 계산
        dist_err = dist - target_dist
        angle_err = center_x - cx

        cmd = Twist()
        
        # 1. 도착 판정 (오차 15cm 이내)
        if abs(dist_err) < 0.15:
            self.actions.cmd_vel_pub.publish(Twist()) # 정지
            return True # 도착 완료!

        # 2. 주행 제어 (P제어)
        cmd.linear.x = min(0.2 * dist_err, 0.15)
        cmd.angular.z = 0.003 * angle_err

        # 안전 장치
        cmd.linear.x = max(min(cmd.linear.x, 0.2), -0.2)
        cmd.angular.z = max(min(cmd.angular.z, 0.5), -0.5)

        self.actions.cmd_vel_pub.publish(cmd)
        return False

    def wait_for_nav(self, timeout=100.0):
        start_time = time.time()
        while not self.actions.is_nav_complete():
            if time.time() - start_time > timeout:
                self.get_logger().warn(f"⏰ 타임아웃! ({timeout}초). 강제로 다음 단계 진행.")
                # Undock은 물리적으로 빠져나왔으면 성공으로 간주하고 True 반환
                return True 
            time.sleep(0.5)
        return self.actions.is_nav_succeeded()
    # ---------------------------------------------------------
    # (E) 미션 루프:
    #     여기서 코드 조건에 따라 action_1 실행
    # ---------------------------------------------------------
    def run_mission_sequence(self):
        self.get_logger().info("🧵 run_mission_sequence loop started")

        while rclpy.ok():
            with self._code_lock:
                code = self._latest_code
                code_time = self._latest_code_time

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

            if code == "afbfcn":
                """
                양쪽방 불, 각자 방 도착후 제자리 회전 , 그러면서 불 찾기 
                FullSequenceTest에서 code == 'afbfcn'일 때 호출.
                ns에 따라 robot2/robot6 이동 루트를 분기.
                """
                self.get_logger().warn("[Mission] code='afbfcn' -> action_1() + fire_search_and_chase()")
                self._last_handled_code = code
                self._last_handled_time = now

                self.is_mission_running = True
                try:
                    # 1) 목적지 이동/도킹해제 루트
                    self.actions.action_1()

                    # 2) 정찰회전 + 화재추적 (ActionLib로 위임)
                    self.actions.fire_search_and_chase(
                        is_target_locked_fn=lambda: self.target_locked,
                        control_to_fire_fn=self.control_robot_to_fire,
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
            
            elif code == "afbncn":
                self.get_logger().warn("[Mission] code='afbccf' (아직 미구현)")
                self._last_handled_code = code
                self._last_handled_time = now

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

if __name__ == '__main__':
    main()