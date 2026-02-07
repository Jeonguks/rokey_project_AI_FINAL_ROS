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
        
        # 상태 변수
        self.is_mission_running = False # 미션 수행중인가 ?  

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

    # ---------------------------------------------------------
    # (E) 미션 루프:
    #     여기서 코드 조건에 따라 action_1 실행
    # ---------------------------------------------------------
    def run_mission_sequence(self):
        self.get_logger().info("🧵 run_mission_sequence loop started")

        while rclpy.ok():
            with self._code_lock:
                code = self._latest_code

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