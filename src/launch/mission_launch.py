from launch import LaunchDescription
from launch_ros.actions import Node

# ------------------------------------------------------------------------------
# [런치 파일 진입점]
# ROS 2 시스템의 구동 시나리오를 정의하는 함수입니다.
# 다중 로봇 시스템(Multi-Robot System)을 고려하여 네임스페이스와 리맵핑을 체계적으로 관리합니다.
# ------------------------------------------------------------------------------
def generate_launch_description():
    # [네임스페이스 설정]
    # 단일 로봇뿐만 아니라 다중 로봇 환경에서도 식별 가능하도록 고유 네임스페이스를 지정합니다.
    # 모든 노드와 토픽은 '/robot6/...' 접두사를 가지게 되어 다른 로봇과 통신 충돌을 방지합니다.
    robot_namespace = 'robot6'

    return LaunchDescription([
        # ----------------------------------------------------------------------
        # 1. Mission Commander (최상위 제어 컨트롤러)
        # 전체 미션의 상태 머신(FSM)을 관리하고 하위 노드들을 조율합니다.
        # ----------------------------------------------------------------------
        Node(
            package='fight_fire',
            executable='mission_commander',
            name='mission_commander',
            namespace=robot_namespace,
            output='screen', # 로그를 터미널 표준 출력으로 표시
            # 글로벌 /tf가 아닌 /robot6/tf를 참조하도록 강제합니다. (좌표계 격리)
            remappings=[
                ('/tf', f'/{robot_namespace}/tf'),
                ('/tf_static', f'/{robot_namespace}/tf_static')
            ]
        ),
        
        # ----------------------------------------------------------------------
        # 2. Perception Node (시각 인지 처리)
        # YOLOv8 추론 및 Depth 처리를 담당하며, Commander에게 시각 정보를 제공합니다.
        # ----------------------------------------------------------------------
        Node(
            package='fight_fire',
            executable='perception_node', 
            name='perception_node',
            namespace=robot_namespace,
            output='screen',
            # Perception 노드 역시 카메라 좌표와 로봇 베이스 간의 TF를 알아야 하므로
            remappings=[
                ('/tf', f'/{robot_namespace}/tf'),
                ('/tf_static', f'/{robot_namespace}/tf_static')
            ]
        ),

        # ----------------------------------------------------------------------
        # 3. Evacuation Node (대피 안내 에이전트)
        # 인명 감지 시 대피소까지 안내하는 독립적인 내비게이션 로직을 수행합니다.
        # ----------------------------------------------------------------------
        Node(
            package='fight_fire',
            executable='evacuation_node',
            name='evacuation_node',
            namespace=robot_namespace,
            output='screen',
            # 내비게이션 수행을 위해 맵 좌표계(TF) 접근이 필요하므로 리맵핑 적용
            remappings=[
                ('/tf', f'/{robot_namespace}/tf'),
                ('/tf_static', f'/{robot_namespace}/tf_static')
            ]
        )
    ])