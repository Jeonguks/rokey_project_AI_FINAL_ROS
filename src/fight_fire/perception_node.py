import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import tf2_ros
import time
import json

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # 설정값
        self.namespace = 'robot2' 
        self.model_path = '/home/rokey/datasets/weights/best_amr_v8n_param_add.pt'
        self.camera_frame_id = ""  
        self.last_process_time = 0.0 
        self.camera_intrinsics = None 
        
        # [데이터 캐시] 타임스탬프 차이 해결을 위해 최신 데이터를 저장
        self.latest_depth_msg = None

        # YOLO 로드
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'✅ YOLO 로드 성공')
        except Exception as e:
            self.get_logger().error(f'❌ YOLO 로드 실패: {e}')

        # QoS 설정 (BEST_EFFORT로 통신 안정성 확보)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 1. Depth 구독 (데이터가 오면 변수에 저장만 함)
        self.depth_sub = self.create_subscription(
            Image, 
            f'/{self.namespace}/oakd/stereo/image_raw', 
            self.depth_callback, 
            qos_profile
        )

        # 2. RGB 구독 (메인 트리거: 이 콜백이 실행될 때 추론 시작)
        self.rgb_sub = self.create_subscription(
            CompressedImage, 
            f'/{self.namespace}/oakd/rgb/image_raw/compressed', 
            self.rgb_callback, 
            qos_profile
        )

        # 3. CameraInfo 구독
        self.info_sub = self.create_subscription(
            CameraInfo, 
            f'/{self.namespace}/oakd/rgb/camera_info', 
            self.info_callback, 
            qos_profile
        )

        # Publisher
        self.detection_pub = self.create_publisher(String, 'perception/detections', 10)
        self.debug_pub = self.create_publisher(CompressedImage, f'/{self.namespace}/yolo_debug/compressed', 10)
        self.log_pub = self.create_publisher(String, '/robot_log', 10)

        self.cv_bridge = CvBridge()
        self.get_logger().info("👀 Perception Node 시작 (동기화 프리 모드)")

    def info_callback(self, msg):
        if self.camera_intrinsics is None:
            K = msg.k
            self.camera_intrinsics = {'fx': K[0], 'fy': K[4], 'cx': K[2], 'cy': K[5]}
            self.camera_frame_id = msg.header.frame_id 
            self.get_logger().info("✅ CameraInfo 수신 완료")

    def depth_callback(self, msg):
        # 최신 Depth 데이터를 지속적으로 갱신
        self.latest_depth_msg = msg

    def rgb_callback(self, rgb_msg):
        # 1. 처리 주기 조절 (초당 10번 정도)
        current_time = time.time()
        if current_time - self.last_process_time < 0.1: 
            return
        self.last_process_time = current_time

        # 2. 필수 데이터 체크
        if self.latest_depth_msg is None:
            self.get_logger().warn("⏳ Depth 대기 중...", throttle_duration_sec=3.0)
            return
        if self.camera_intrinsics is None:
            return

        # 3. 메인 로직 실행
        self.process_perception(rgb_msg, self.latest_depth_msg)

    def process_perception(self, rgb_msg, depth_msg):
        try:
            # 이미지 변환
            frame = self.cv_bridge.compressed_imgmsg_to_cv2(rgb_msg, "bgr8")
            current_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_h, depth_w = current_depth.shape[:2]

            # YOLO 추론
            results = self.model(frame, verbose=False)
            detected_objects_list = []

            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    if conf < 0.7: continue 

                    class_name = result.names[cls_id]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    # ROI 내 거리 평균 계산
                    box_w, box_h = x2 - x1, y2 - y1
                    sx1, sx2 = max(0, int(cx - box_w*0.1)), min(depth_w, int(cx + box_w*0.1))
                    sy1, sy2 = max(0, int(cy - box_h*0.1)), min(depth_h, int(cy + box_h*0.1))
                    
                    depth_roi = current_depth[sy1:sy2, sx1:sx2]
                    valid_depth = depth_roi[depth_roi > 0]
                    dist_m = float(np.median(valid_depth) / 1000.0) if len(valid_depth) > 0 else 0.0
                    
                    obj_data = {
                        "class": class_name, "conf": round(conf, 2), "dist": round(dist_m, 3),
                        "cx": cx, "cy": cy, "width": box_w, "height": box_h
                    }
                    detected_objects_list.append(obj_data)

                    # 시각화
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{class_name} {dist_m:.2f}m", (x1, y1-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 결과 전송
            if detected_objects_list:
                self.detection_pub.publish(String(data=json.dumps(detected_objects_list)))
                self.get_logger().info(f"🔎 Detected: {len(detected_objects_list)} items 이름: {class_name}, conf: {conf:.2f}")

            # 디버그 이미지 발행
            debug_msg = self.cv_bridge.cv2_to_compressed_imgmsg(frame)
            debug_msg.header.stamp = rgb_msg.header.stamp
            debug_msg.header.frame_id = self.camera_frame_id
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error in process_perception: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()