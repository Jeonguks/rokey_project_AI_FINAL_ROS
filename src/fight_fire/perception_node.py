import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs
import time
import json
import message_filters

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        self.namespace = 'robot6' 
        self.model_path = '/home/rokey/datasets/weights/best_amr_v8n_param_add.pt'
        self.camera_frame_id = ""  
        self.last_process_time = 0.0 
        self.camera_intrinsics = None 

        # ---------------------------------------------------------
        # 1. YOLO 모델 로드
        # ---------------------------------------------------------
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'✅ YOLO 모델 로드 성공: {self.model_path}')
        except Exception as e:
            self.get_logger().error(f'❌ YOLO 모델 로드 실패: {e}')

        # ---------------------------------------------------------
        # 2. TF (좌표 변환용)
        # ---------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------------------------------------------------
        # 3. 통신 설정 (Subscriber)
        # ---------------------------------------------------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # RGB & Depth 동기화 구독
        self.rgb_sub = message_filters.Subscriber(
            self, CompressedImage, f'/{self.namespace}/oakd/rgb/image_raw/compressed', qos_profile=qos_profile)
        self.depth_sub = message_filters.Subscriber(
            self, Image, f'/{self.namespace}/oakd/stereo/image_raw', qos_profile=qos_profile)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=1, slop=0.5
        )
        self.sync.registerCallback(self.sync_callback)

        # 카메라 정보 구독
        self.create_subscription(CameraInfo, f'/{self.namespace}/oakd/rgb/camera_info', self.info_callback, qos_profile)

        # ---------------------------------------------------------
        # 4. 통신 설정 (Publisher) - 로직 제거 후 정보만 전달
        # ---------------------------------------------------------
        # [핵심] 감지된 모든 객체의 정보를 JSON으로 묶어서 보냄
        # 예: [{"class": "fire", "dist": 1.5, "cx": 320, "map_x": 2.1, "map_y": 3.5}, ...]
        self.detection_pub = self.create_publisher(String, 'perception/detections', 10)
        
        # 디버깅용 이미지
        self.debug_pub = self.create_publisher(CompressedImage, f'/{self.namespace}/yolo_debug', 10)
        
        # RViz 시각화용 마커 (Down 객체 등 표시)
        self.marker_pub = self.create_publisher(Marker, f'/{self.namespace}/detection_marker', 10)

        self.cv_bridge = CvBridge()
        self.get_logger().info("👀 Perception Node Ready (Pure Sensor Mode)")
        self.get_logger().info("1111111111111111111111")

    def info_callback(self, msg):
        if self.camera_intrinsics is None:
            K = msg.k
            self.camera_intrinsics = {'fx': K[0], 'fy': K[4], 'cx': K[2], 'cy': K[5]}
            self.camera_frame_id = msg.header.frame_id 

    def sync_callback(self, rgb_msg, depth_msg):

        self.get_logger().info(f"sync callback 진입 ---------------------------------")
        current_time = time.time()
        # FPS 제한 (시스템 부하 방지)
        if current_time - self.last_process_time < 0.12: # 약 8 FPS
            return
        self.last_process_time = current_time

        if self.camera_intrinsics is None: return

        try:
            # 1. 이미지 디코딩
            frame = self.cv_bridge.compressed_imgmsg_to_cv2(rgb_msg, "bgr8")
            if frame is None: return

            # 2. Depth 디코딩
            current_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_h, depth_w = current_depth.shape[:2]

            # 3. YOLO 추론
            results = self.model(frame, verbose=False)
            
            detected_objects_list = [] # 발행할 정보 리스트

            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = result.names[cls_id]
                    
                    if conf < 0.4: continue # 신뢰도 컷

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    # 시각화 (박스 그리기)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{class_name} {conf:.2f}"
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # -----------------------------------------------------
                    # 거리 및 좌표 계산 (Sensor Fusion)
                    # -----------------------------------------------------
                    dist_m = 0.0
                    map_point = None

                    # 박스 중앙부의 Depth 평균값 추출
                    box_w, box_h = x2 - x1, y2 - y1
                    sx1, sx2 = max(0, int(cx - box_w*0.1)), min(depth_w, int(cx + box_w*0.1))
                    sy1, sy2 = max(0, int(cy - box_h*0.1)), min(depth_h, int(cy + box_h*0.1))

                    depth_roi = current_depth[sy1:sy2, sx1:sx2]
                    valid_depth = depth_roi[depth_roi > 0]

                    if len(valid_depth) > 0:
                        dist_m = float(np.median(valid_depth) / 1000.0) # mm -> meter
                        
                        # 3D 좌표 변환 (Map Frame 기준)
                        map_point = self.get_map_coordinate(cx, cy, dist_m)
                    
                    # -----------------------------------------------------
                    # 데이터 패키징
                    # -----------------------------------------------------
                    obj_data = {
                        "class": class_name,
                        "conf": round(conf, 2),
                        "dist": round(dist_m, 3), # 거리가 없으면 0.0
                        "cx": cx,                 # 화면 중심 x좌표 (회전 제어용)
                        "cy": cy,
                        "width": box_w,           # 박스 크기
                        "height": box_h
                    }

                    # 맵 좌표가 있다면 추가 (내비게이션 목표용)
                    # if map_point:
                    #     obj_data["map_x"] = round(map_point.point.x, 3)
                    #     obj_data["map_y"] = round(map_point.point.y, 3)
                        
                    #     # RViz에 마커 띄우기 (옵션)
                    #     self.publish_marker(map_point, class_name)

                    detected_objects_list.append(obj_data)

            # 4. 결과 발행 (JSON 문자열)
            # 받는 쪽 예시: data = json.loads(msg.data) -> if data[0]['class'] == 'fire': ...
            if detected_objects_list:
                json_str = json.dumps(detected_objects_list)
                self.detection_pub.publish(String(data=json_str))
                # 로그는 너무 자주 찍히면 보기 힘드니 필요시 주석 해제
                # self.get_logger().info(f"Broadcast: {json_str}")

            # 5. 디버그 이미지 발행
            # small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
            # out_msg = self.cv_bridge.cv2_to_compressed_imgmsg(small_frame)
            out_msg = self.cv_bridge.cv2_to_compressed_imgmsg(frame)
            self.debug_pub.publish(out_msg)
            self.get_logger().info("222222222222222222222222222222222")

        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

    def get_map_coordinate(self, u, v, dist_m):
        """이미지 픽셀(u,v)와 거리(dist)를 이용해 Map 기준 좌표 계산"""
        if dist_m <= 0: return None

        # 1. 카메라 좌표계 (3D)
        z_cam = dist_m
        x_cam = (u - self.camera_intrinsics['cx']) * z_cam / self.camera_intrinsics['fx']
        y_cam = (v - self.camera_intrinsics['cy']) * z_cam / self.camera_intrinsics['fy']

        point_cam = PointStamped()
        point_cam.header.frame_id = self.camera_frame_id if self.camera_frame_id else "oakd_rgb_camera_optical_frame"
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.point.x = x_cam
        point_cam.point.y = y_cam
        point_cam.point.z = z_cam

        # 2. TF 변환 (Camera -> Map)
        try:
            target_frame = "map"
            transform = self.tf_buffer.lookup_transform(
                target_frame, 
                point_cam.header.frame_id, 
                rclpy.time.Time(seconds=0)
            )
            point_map = tf2_geometry_msgs.do_transform_point(point_cam, transform)
            return point_map
        except Exception:
            return None

    def publish_marker(self, map_point, class_name):
        """RViz에서 위치를 확인할 수 있게 마커 발행"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_objects"
        marker.id = hash(class_name) % 10000 # 고유 ID 생성
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = map_point.point.x
        marker.pose.position.y = map_point.point.y
        marker.pose.position.z = 0.5 
        
        marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2
        
        # 색상 (클래스별 다르게)
        marker.color.a = 0.8
        if class_name == 'fire':
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0 # 빨강
        elif class_name == 'stand':
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0 # 초록
        elif class_name == 'down':
            marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0 # 파랑
        else:
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0 # 노랑
            
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()