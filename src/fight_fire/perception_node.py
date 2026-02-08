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
import tf2_geometry_msgs
import time
import json
import message_filters


'''
[
  {
    "class": "fire",
    "conf": 0.91,
    "dist": 1.42,
    "cx": 312,
    "cy": 241,
    "width": 88,
    "height": 95
  },
  {
    "class": "stand",
    "conf": 0.83,
    "dist": 2.05,
    "cx": 520,
    "cy": 260,
    "width": 60,
    "height": 130
  }
]


'''

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        self.namespace = 'robot6' 
        self.model_path = '/home/rokey/datasets/weights/best_amr_v8n_param_add.pt'
        self.camera_frame_id = ""  
        self.last_process_time = 0.0 
        self.camera_intrinsics = None 
        
        # 로그 Publisher
        self.log_pub = self.create_publisher(String, '/robot_log', 10)

        # YOLO 로드
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'✅ YOLO 로드 성공')
        except Exception as e:
            self.get_logger().error(f'❌ YOLO 로드 실패: {e}')

        # TF 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------------------------------------------------
        # [핵심] QoS 설정 - "호환성 최강" 설정으로 변경
        # ---------------------------------------------------------
        # Reliability: Best Effort (데이터 좀 빠져도 받음)
        # Durability: Volatile (지나간 건 안 받음)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.get_logger().info("⏳ [1단계] 이미지 구독 시작... (CameraInfo 대기중)")

        # 구독 설정
        self.rgb_sub = message_filters.Subscriber(
            self, CompressedImage, f'/{self.namespace}/oakd/rgb/image_raw/compressed', qos_profile=qos_profile)
        self.depth_sub = message_filters.Subscriber(
            self, Image, f'/{self.namespace}/oakd/stereo/image_raw', qos_profile=qos_profile)

        # ---------------------------------------------------------
        # [수정] 큐 사이즈 10 + 시간 오차(slop) 2.0초로 대폭 증가
        # ---------------------------------------------------------
        # 서로 2초나 차이나도 일단 묶어보라는 뜻 (싱크 문제 해결용)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=20, # 넉넉하게
            slop=2.0       # 2초까지 허용
        )
        self.sync.registerCallback(self.sync_callback)

        # CameraInfo 구독
        self.create_subscription(CameraInfo, f'/{self.namespace}/oakd/rgb/camera_info', self.info_callback, qos_profile)

        # Publisher
        self.detection_pub = self.create_publisher(String, 'perception/detections', 10)
        # [중요] RViz용 압축 이미지 토픽
        self.debug_pub = self.create_publisher(CompressedImage, f'/{self.namespace}/yolo_debug/compressed', 10)
        self.marker_pub = self.create_publisher(Marker, f'/{self.namespace}/detection_marker', 10)

        self.cv_bridge = CvBridge()
        self.publish_log("👀 Perception Node 시작됨 (디버깅 모드)")

    def publish_log(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.log_pub.publish(msg)

    def info_callback(self, msg):
        # CameraInfo가 들어오는지 확인하는 로그
        if self.camera_intrinsics is None:
            self.get_logger().info("✅ [2단계] CameraInfo 수신 성공! (이제 sync_callback만 되면 됨)")
            K = msg.k
            self.camera_intrinsics = {'fx': K[0], 'fy': K[4], 'cx': K[2], 'cy': K[5]}
            self.camera_frame_id = msg.header.frame_id 

    def sync_callback(self, rgb_msg, depth_msg):
        # 여기가 실행되는지 확인하는 로그 (너무 많이 뜨면 렉걸리니 10번에 1번만)
        # self.get_logger().info(f"⚡ [3단계] Sync 성공! (RGB: {rgb_msg.header.stamp.sec}, Depth: {depth_msg.header.stamp.sec})")
        
        current_time = time.time()
        if current_time - self.last_process_time < 0.1: return
        self.last_process_time = current_time

        # CameraInfo 없으면 진행 불가
        if self.camera_intrinsics is None:
            self.get_logger().warn("⚠️ 이미지는 들어오는데 CameraInfo가 아직 없습니다!")
            return

        try:
            frame = self.cv_bridge.compressed_imgmsg_to_cv2(rgb_msg, "bgr8")
            if frame is None: return

            current_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_h, depth_w = current_depth.shape[:2]

            # YOLO 추론
            results = self.model(frame, verbose=False)
            
            detected_objects_list = []

            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = result.names[cls_id]
                    
                    # [수정 1] 커트라인을 0.4 -> 0.2로 낮춤 (더 민감하게)
                    if conf < 0.7: continue 

                    # [수정 2] 감지된 건 일단 무조건 터미널에 출력 (디버깅용)
                    self.get_logger().info(f"🧐 감지됨! -> 이름: {class_name}, 점수: {conf:.2f}")

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{class_name} {conf:.2f}"
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # 거리 계산 (간략화)
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

            # JSON 발행
            if detected_objects_list:
                json_str = json.dumps(detected_objects_list)
                self.detection_pub.publish(String(data=json_str))
                # 감지되면 로그 한번 찍기
                # self.get_logger().info(f"🔎 감지됨: {json_str}")

            # [이미지 발행]
            debug_out = self.cv_bridge.cv2_to_compressed_imgmsg(frame)
            debug_out.header.stamp = rgb_msg.header.stamp 
            debug_out.header.frame_id = self.camera_frame_id
            
            self.debug_pub.publish(debug_out)

        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

    # (TF 함수 생략 가능 - 에러 없으면)
    def get_map_coordinate(self, u, v, dist_m):
        return None 

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()