import rclpy
from geometry_msgs.msg import PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult 
import time

# ------------------------------------------------------------------------------
# [NavigationClient 클래스]
# TurtleBot4의 내비게이션 기능(Nav2)을 캡슐화(Encapsulation)하여,
# 상위 레벨(Mission Commander)에서 비즈니스 로직에만 집중할 수 있도록 돕는 추상화 계층입니다.
# 
# 역할:
# 1. Nav2 Action Client 관리 (이동 명령 전송, 취소)
# 2. 로봇의 초기 위치 추정 (Initial Pose Estimation) 초기화
# 3. 주요 포인트 관리
# ------------------------------------------------------------------------------
class NavigationClient:
    def __init__(self):
        # 멀티 로봇 환경을 고려하여 네임스페이스를 명시적으로 지정
        self.namespace = 'robot6' 
        
        # TurtleBot4Navigator 인스턴스 생성 (Nav2 API 래퍼)
        self.navigator = TurtleBot4Navigator(namespace=self.namespace)

        # [안전 장치] 초기화 시 로봇이 도킹되어 있지 않다면 강제로 도킹 시도
        # 배터리 충전 상태 확보 및 시작 위치의 불확실성 제거 목적
        if not self.navigator.getDockedStatus():
            self.navigator.info('초기화 전 도킹 상태가 아닙니다. 도킹을 시도합니다.')
            self.navigator.dock()

        # [초기 위치 설정 (AMCL Initialization)]
        # 로봇이 맵 상의 어디에 있는지 확률적 위치 추정 알고리즘(AMCL)에 알려줍니다.
        # 시뮬레이션 환경의 고정된 시작 좌표를 사용합니다.
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        # 쿼터니언(Quaternion) 및 좌표 설정
        initial_pose.pose.position.x = -0.117279
        initial_pose.pose.position.y = 0.019437
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = -0.771297
        initial_pose.pose.orientation.w = 0.636474

        self.navigator.info('AMCL 초기 위치(Initial Pose)를 설정합니다...')
        self.navigator.setInitialPose(initial_pose)
        
        # [중요] Nav2 활성화 대기 로직 변경
        # 원래는 waitUntilNav2Active()로 AMCL의 활성화를 무한정 기다려야 하나,
        # 시뮬레이터 통신 지연으로 인한 Deadlock 방지를 위해 3초 대기 후 강제 진행(Force Proceed)합니다.
        time.sleep(3.0) 
        self.navigator.info('Navigation Client 초기화 완료 (Active 상태 가정).')

    # --------------------------------------------------------------------------
    # [이동 명령 API] - 비동기(Asynchronous) 호출 방식 사용
    # startToPose()는 명령만 내리고 즉시 리턴하므로, 상위 노드에서 완료 여부를 체크해야 함.
    # --------------------------------------------------------------------------

    def undock(self):
        """충전 스테이션에서 분리(Undock)합니다."""
        self.navigator.undock()

    def is_task_complete(self):
        """현재 수행 중인 Nav2 태스크가 종료되었는지(성공/실패/취소 포함) 확인"""
        return self.navigator.isTaskComplete()
    
    def is_task_succeeded(self):
        """태스크가 '성공적'으로 끝났는지 결과(Result) 코드를 검증"""
        return self.navigator.getResult() == TaskResult.SUCCEEDED
    
    def stop(self):
        """비상 정지: 현재 수행 중인 모든 내비게이션 태스크 취소"""
        self.navigator.cancelTask()

    def move_to_wp1(self):
        """
        경유지 1 (WayPoint 1)로 이동 명령 발행.
        TurtleBot4Directions 헬퍼를 사용하여 방향(Orientation)을 쉽게 설정.
        """
        # (x, y) 좌표와 바라볼 방향(NORTH)을 지정하여 목표 포즈 생성
        goal_pose = self.navigator.getPoseStamped([0.6461, 2.7294], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal_pose)

    def move_to_wp2(self):
        """최종 목적지 (WayPoint 2 - B구역 탕비실)로 이동 명령 발행"""
        goal_pose = self.navigator.getPoseStamped([2.0303, 2.1183], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(goal_pose)

    def spin_360(self):
        """
        제자리 회전(Spin) 동작 수행.
        정찰(Reconnaissance) 및 주변 환경 스캔 목적.
        """
        self.navigator.info('B구역 도착. 정찰을 위해 360도 회전을 수행합니다.')
        # 6.28 라디안 = 360도, 시간 제한 10초 부여
        self.navigator.spin(spin_dist=6.28, time_allowance=10)

    def move_to_friend(self):
        """협력 미션: 동료 로봇(Robot2) 지원 위치로 이동"""
        self.navigator.info('동료 지원 요청 수신. 지정된 지원 좌표로 이동합니다.')
        goal_pose = self.navigator.getPoseStamped([1.5, 1.5], TurtleBot4Directions.SOUTH)
        self.navigator.startToPose(goal_pose)

    def dock(self):
        """복귀 및 자동 충전 시퀀스 실행"""
        self.navigator.dock()