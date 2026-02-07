#!/usr/bin/env python3
import sys
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # 네비게이션 코드는 싹 빠지고, "좌표 발행자"만 사용
        self.target_pub = self.create_publisher(PointStamped, "/target_point", 10)

        # 토픽 설정
        ns = self.get_namespace()
        self.rgb_topic = f"{ns}/oakd/rgb/image_raw/compressed"
        self.depth_topic = f"{ns}/oakd/stereo/image_raw"
        self.info_topic = f"{ns}/oakd/rgb/camera_info"

        # 데이터 변수
        self.depth_image = None
        self.rgb_image = None
        self.display_image = None
        self.K = None
        self.camera_frame = None

        # YOLO 모델 로드
        self.model = None
        try:
            self.model = YOLO("/home/rokey/rokey_ws/test_code/amr_default_best.pt")
            self.get_logger().info("✅ YOLOv8 모델 로드 성공")
        except Exception as e:
            self.get_logger().error(f"❌ YOLO 모델 로드 실패: {e}")

        # TF2 (카메라 좌표 -> 맵 좌표 변환용)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 구독
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, qos_profile_sensor_data)

        # 주기 처리
        self.create_timer(0.1, self.process_and_publish)

        # GUI 스레드
        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

    def camera_info_callback(self, msg: CameraInfo):
        with self.lock:
            self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self.lock:
                self.depth_image = depth
                self.camera_frame = msg.header.frame_id
        except Exception:
            pass

    def rgb_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            with self.lock:
                self.rgb_image = rgb
        except Exception:
            pass

    def publish_target(self, u: int, v: int, z: float, frame_id: str):
        """좌표 변환 후 토픽 발행"""
        if self.K is None:
            return

        fx, fy = float(self.K[0, 0]), float(self.K[1, 1])
        cx, cy = float(self.K[0, 2]), float(self.K[1, 2])

        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        Z = z

        pt_camera = PointStamped()
        pt_camera.header.stamp = Time().to_msg()
        pt_camera.header.frame_id = frame_id
        pt_camera.point.x = float(X)
        pt_camera.point.y = float(Y)
        pt_camera.point.z = float(Z)

        try:
            target_frame = "map"
            pt_map = self.tf_buffer.transform(
                pt_camera,
                target_frame,
                timeout=Duration(seconds=0.5),
            )

            self.target_pub.publish(pt_map)
            self.get_logger().info(
                f"🎯 타겟 발견! 좌표 전송: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})"
            )
        except Exception as e:
            self.get_logger().warn(f"TF 변환 실패: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.process_target_logic(x, y)

    def process_target_logic(self, u: int, v: int):
        with self.lock:
            depth = None if self.depth_image is None else self.depth_image
            frame_id = self.camera_frame

        if depth is None or frame_id is None or self.K is None:
            return

        h, w = depth.shape[:2]
        if not (0 <= v < h and 0 <= u < w):
            return

        try:
            z = float(depth[v, u]) / 1000.0  # mm -> m (센서가 mm면 이게 맞음)
        except Exception:
            return

        if 0.1 < z < 5.0:
            self.publish_target(u, v, z, frame_id)

    def process_and_publish(self):
        with self.lock:
            rgb = None if self.rgb_image is None else self.rgb_image.copy()
            depth = None if self.depth_image is None else self.depth_image.copy()

        if rgb is None or depth is None or self.model is None:
            return

        results = self.model(rgb, verbose=False)[0]
        annotated = rgb.copy()

        for det in results.boxes:
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())
            conf = float(det.conf[0])
            cls_id = int(det.cls[0])
            label = self.model.names.get(cls_id, str(cls_id))

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                f"{label} {conf:.2f}",
                (x1, max(0, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )

            if label.lower() == "car" and conf > 0.5:
                cx, cy = int((x1 + x2) // 2), int((y1 + y2) // 2)
                cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)

                # 발견 즉시 좌표 처리 로직 호출
                self.process_target_logic(cx, cy)

        # GUI용 이미지 생성 (깊이 맵 합치기)
        d_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        d_color = cv2.applyColorMap(d_norm.astype(np.uint8), cv2.COLORMAP_JET)

        if annotated.shape[:2] != d_color.shape[:2]:
            d_color = cv2.resize(d_color, (annotated.shape[1], annotated.shape[0]))

        disp = np.hstack((annotated, d_color))

        with self.lock:
            self.display_image = disp

    def gui_loop(self):
        cv2.namedWindow("Detector", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Detector", self.mouse_callback)

        while rclpy.ok():
            with self.lock:
                img = None if self.display_image is None else self.display_image.copy()

            if img is not None:
                cv2.imshow("Detector", img)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                cv2.waitKey(10)


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
