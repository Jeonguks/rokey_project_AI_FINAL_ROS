import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Bool, Float32MultiArray
import time
import math
from enum import Enum

# 상태 정의
class State(Enum):
    IDLE = 0            
    FOLLOW_WALL = 1     
    SCAN_TURN_RIGHT = 2 # 우측 90도 회전 (IMU 감시)
    SCAN_LOOK = 3       
    SCAN_TURN_BACK = 4  # 좌측 90도 회전 (IMU 감시)
    APPROACH_FIRE = 5   
    EXTINGUISH_FIRE = 6 

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')
        self.state = State.IDLE
        
        # --- 파라미터 ---
        self.scan_interval = 4.0  
        self.scan_duration = 2.0  
        self.target_angle = 1.50 
        self.turn_speed = 0.5     
        
        # [PD 제어 파라미터]
        self.target_dist = 0.9    # 목표 거리: 90cm
        self.p_gain = 2.0         # P 게인 (현재 오차에 반응)
        self.d_gain = 5.0         # D 게인 (오차 변화율에 반응, 진동 억제)
        self.prev_error = 0.0     # D 제어용 이전 오차 저장
        # ----------------

        self.state_start_time = 0.0
        self.latest_scan = None
        self.fire_info = None
        
        # IMU 관련 변수
        self.current_yaw = 0.0
        self.start_yaw = 0.0
        
        qos = QoSProfile(depth=10)
        
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.create_subscription(Imu, 'imu', self.imu_callback, qos)
        self.create_subscription(Bool, 'enable_wall_follower', self.enable_callback, 10)
        self.create_subscription(Float32MultiArray, 'fire_info', self.fire_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Smart PD Wall Follower (Dist: {self.target_dist}m) Ready.')

    def enable_callback(self, msg):
        if msg.data:
            self.state = State.FOLLOW_WALL
            self.state_start_time = time.time()
            self.prev_error = 0.0  # 활성화 시 오차 초기화
            self.get_logger().info('START: PD Wall Following with IMU Patrol')
        else:
            self.state = State.IDLE
            self.stop_robot()

    def scan_callback(self, msg):
        self.latest_scan = msg

    def fire_callback(self, msg):
        self.fire_info = msg.data

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw = yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if self.state == State.IDLE:
            return

        cmd = Twist()
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time

        if self.fire_info and self.fire_info[2] == 1.0:
            if self.state not in [State.APPROACH_FIRE, State.EXTINGUISH_FIRE]:
                self.get_logger().warn('!!! FIRE DETECTED !!!')
                self.state = State.APPROACH_FIRE

        if self.state == State.FOLLOW_WALL:
            if elapsed_time > self.scan_interval:
                self.get_logger().info('Patrol: Start Turning Right (IMU)...')
                self.state = State.SCAN_TURN_RIGHT
                self.start_yaw = self.current_yaw
            else:
                cmd = self.get_wall_follow_cmd()

        elif self.state == State.SCAN_TURN_RIGHT:
            angle_diff = self.normalize_angle(self.start_yaw - self.current_yaw)
            if abs(angle_diff) < self.target_angle:
                cmd.angular.z = -self.turn_speed
            else:
                self.state = State.SCAN_LOOK
                self.state_start_time = current_time

        elif self.state == State.SCAN_LOOK:
            if elapsed_time > self.scan_duration:
                self.state = State.SCAN_TURN_BACK
                self.start_yaw = self.current_yaw
            else:
                cmd.linear.x = 0.0 
                cmd.angular.z = 0.0

        elif self.state == State.SCAN_TURN_BACK:
            angle_diff = self.normalize_angle(self.current_yaw - self.start_yaw)
            if abs(angle_diff) < self.target_angle:
                cmd.angular.z = self.turn_speed
            else:
                self.state = State.FOLLOW_WALL
                self.state_start_time = current_time
                self.prev_error = 0.0  # 복귀 시 오차 초기화

        self.cmd_pub.publish(cmd)

    def get_wall_follow_cmd(self):
        cmd = Twist()
        if self.latest_scan is None:
            return cmd
            
        # 1. 데이터 전처리 (0.05m 이하 데이터는 노이즈/사각지대로 간주하여 무시)
        scan_data = [x if x > 0.05 else 10.0 for x in self.latest_scan.ranges]
        deg_to_idx = len(scan_data) / 360.0
        
        # 2. 센서 데이터 추출
        # 정면 20도 범위
        front_dist = min(scan_data[0 : int(10 * deg_to_idx)] + scan_data[int(350 * deg_to_idx) : ])
        # 왼쪽 사선 30도~60도 범위 (돌진 방지 및 조기 감지용)
        left_diag_dist = min(scan_data[int(30 * deg_to_idx) : int(60 * deg_to_idx)])
        # 자세 제어용 측면 및 전사선 데이터
        left_side_dist = min(scan_data[int(85 * deg_to_idx) : int(95 * deg_to_idx)])
        left_front_dist = scan_data[int(45 * deg_to_idx)]

        # --- 파라미터 설정 ---
        target_dist = 0.9    # 벽과의 목표 거리 (90cm)
        stop_dist = 0.9      # 충돌 방지 및 긴급 회전 거리 (90cm로 강화)
        error = target_dist - left_side_dist  # 현재 오차

        # [1단계: 최우선] 돌진 방지 및 긴급 회전 (Emergency)
        # 정면이나 왼쪽 대각선에 벽이 0.9m 이내로 들어오면 즉시 대응
        if front_dist < stop_dist or left_diag_dist < (stop_dist * 0.8):
            self.get_logger().warn(f"Collision Danger! (F:{front_dist:.2f}m). Stopping and Turning Right.")
            cmd.linear.x = 0.0      # 전진 중단
            cmd.angular.z = -0.7    # 오른쪽으로 크게 제자리 회전
            self.prev_error = error  # 에러 갱신
            return cmd

        # [2단계] 벽 탐색 (Searching)
        # 왼쪽에 벽이 너무 멀리 있다면(1.5m 이상), 왼쪽으로 머리를 돌려 벽을 찾음
        if left_side_dist > 1.5:
            cmd.linear.x = 0.1
            cmd.angular.z = 0.4
            self.prev_error = error
            return cmd

        # [3단계] 자세 정렬 (Orientation Aligning)
        # 벽을 향해 파고드는 경우 (사선 < 측면) -> 우회전
        if left_front_dist < left_side_dist - 0.05:
            cmd.linear.x = 0.08
            cmd.angular.z = -0.4
            self.prev_error = error
            return cmd
        # 벽에서 멀어지는 경우 (사선 > 측면) -> 좌회전
        elif left_front_dist > left_side_dist + 0.05:
            cmd.linear.x = 0.08
            cmd.angular.z = 0.4
            self.prev_error = error
            return cmd

        # [4단계] 거리 유지 제어 (PD Control)
        # 로봇이 대략 평행할 때 작동하여 부드러운 거리 유지를 수행
        derivative = (error - self.prev_error) / 0.1 # 제어 주기 0.1초 기준
        
        cmd.linear.x = 0.22 
        # P 항(error)과 D 항(derivative)의 합으로 회전 속도 결정
        cmd.angular.z = (self.p_gain * error) + (self.d_gain * derivative)

        # 다음 루프를 위해 현재 오차 저장
        self.prev_error = error

        # 회전 속도 제한 (안전장치)
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)
            
        return cmd

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()