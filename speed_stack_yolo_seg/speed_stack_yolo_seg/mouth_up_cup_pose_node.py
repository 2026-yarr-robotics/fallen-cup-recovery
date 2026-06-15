#!/usr/bin/env python3
"""
mouth_up_cup_pose_node.py

Hand-eye 비전 노드: 넓은 입구(mouth)가 천장/카메라를 향해 똑바로 선 컵
(YOLO 클래스 'mouth-up-cup')을 검출해서, 컵 윗면(입구) 중심의 3D 좌표를
**카메라 광학 좌표계 그대로** PoseStamped 로 발행한다.

base_link 변환은 하지 않는다(로봇 노드 place_mouth_up_cup 가 자기 EE FK 로 변환).
→ 이 노드는 MoveItPy 가 필요 없고 카메라/depth 만 있으면 동작.

Output:
  /mouth_up_cup/grasp_pose  : geometry_msgs/PoseStamped (camera optical frame)
       pose.position = 컵 윗면 중심의 deproject 3D (m). orientation = identity.
       header.frame_id = color optical frame (camera_info 의 frame_id).
  /mouth_up_cup/debug_image : sensor_msgs/Image (검출/선택 시각화)

선택 규칙: target_class 의 mask 중 면적이 가장 큰(=가장 가깝/또렷한) 한 개.

전제:
  - RealSense color/aligned_depth/camera_info 토픽이 떠 있음.
  - YOLO weight 가 mouth-up-cup 클래스를 포함.
"""

import time

import cv2
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

from ultralytics import YOLO


def imgmsg_to_cv2(msg, desired_encoding="passthrough"):
    """sensor_msgs/Image → cv2/numpy (cv_bridge 없이, numpy ABI 충돌 회피)."""
    h, w = msg.height, msg.width
    enc = msg.encoding

    if enc == "16UC1":
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
    elif enc == "32FC1":
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
    elif enc == "mono8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
    elif enc in ("bgr8", "rgb8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    elif enc in ("bgra8", "rgba8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
    else:
        raise ValueError(f"Unsupported encoding: {enc}")

    if desired_encoding == "passthrough" or desired_encoding == enc:
        return arr.copy()
    if desired_encoding == "bgr8":
        if enc == "rgb8":
            return arr[:, :, ::-1].copy()
        if enc == "bgra8":
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        if enc == "rgba8":
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        if enc == "mono8":
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"Cannot convert {enc} → {desired_encoding}")


def cv2_to_imgmsg(image, encoding="bgr8"):
    msg = Image()
    h, w = image.shape[:2]
    msg.height = h
    msg.width = w
    msg.encoding = encoding
    msg.is_bigendian = 0
    if encoding in ("bgr8", "rgb8"):
        msg.step = w * 3
    elif encoding == "mono8":
        msg.step = w
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")
    msg.data = image.tobytes()
    return msg


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ["true", "1", "yes", "y"]
    return bool(value)


class MouthUpCupPoseNode(Node):
    def __init__(self):
        super().__init__("mouth_up_cup_pose_node")

        self.declare_parameter("weights_path", "")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info")

        self.declare_parameter("grasp_pose_topic", "/mouth_up_cup/grasp_pose")
        self.declare_parameter("debug_image_topic", "/mouth_up_cup/debug_image")

        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("half", False)

        self.declare_parameter("target_class_name", "mouth-up-cup")
        self.declare_parameter("min_mask_area", 300.0)
        # depth median 윈도우 (px). 컵 입구 가운데는 빈 공간이라 depth 가 멀게
        # 잡힐 수 있으므로 윈도우를 키워 rim(테두리) depth 가 섞이게 한다.
        self.declare_parameter("depth_window", 9)

        self.weights_path = str(self.get_parameter("weights_path").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.grasp_pose_topic = str(self.get_parameter("grasp_pose_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)

        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.device = str(self.get_parameter("device").value)
        self.half = as_bool(self.get_parameter("half").value)

        self.target_class_name = str(self.get_parameter("target_class_name").value)
        self.min_mask_area = float(self.get_parameter("min_mask_area").value)
        self.depth_window = int(self.get_parameter("depth_window").value)

        if self.weights_path == "":
            raise RuntimeError("weights_path is empty.")

        if self.device != "cpu" and not torch.cuda.is_available():
            self.get_logger().warn("CUDA 미사용 가능 → CPU 폴백")
            self.device = "cpu"
            self.half = False
        if self.device == "cpu":
            self.half = False

        self.last_depth_m = None
        self.fx = self.fy = self.cx = self.cy = None
        self.optical_frame = None

        self.get_logger().info(f"Loading YOLO model: {self.weights_path}")
        self.model = YOLO(self.weights_path)
        try:
            self.model.fuse()
        except Exception as e:
            self.get_logger().warn(f"model.fuse() skipped: {e}")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Image, self.image_topic, self.image_callback, qos)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, qos)

        self.pose_pub = self.create_publisher(
            PoseStamped, self.grasp_pose_topic, 10)
        self.debug_pub = self.create_publisher(
            Image, self.debug_image_topic, 10)

        self.get_logger().info("mouth_up_cup_pose_node started.")
        self.get_logger().info(f"  image_topic : {self.image_topic}")
        self.get_logger().info(f"  grasp_pose  : {self.grasp_pose_topic} (camera frame)")
        self.get_logger().info(f"  target_class: '{self.target_class_name}'")
        self.get_logger().info(f"  model classes: {getattr(self.model, 'names', None)}")

    # ── depth / camera info ──────────────────────────────────
    def depth_callback(self, msg: Image):
        try:
            depth = imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"depth conversion failed: {e}")
            return
        if msg.encoding == "16UC1":
            self.last_depth_m = depth.astype(np.float32) * 0.001
        elif msg.encoding == "32FC1":
            self.last_depth_m = depth.astype(np.float32)
        else:
            self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        if msg.header.frame_id:
            self.optical_frame = msg.header.frame_id

    def get_depth_at_pixel(self, u, v, window=None):
        if self.last_depth_m is None:
            return None
        if window is None:
            window = self.depth_window
        h, w = self.last_depth_m.shape[:2]
        u = int(round(u))
        v = int(round(v))
        if u < 0 or u >= w or v < 0 or v >= h:
            return None
        r = window // 2
        x0, x1 = max(0, u - r), min(w, u + r + 1)
        y0, y1 = max(0, v - r), min(h, v + r + 1)
        patch = self.last_depth_m[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch)]
        valid = valid[valid > 0.05]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def deproject_pixel_to_3d(self, u, v, z):
        if None in (self.fx, self.fy, self.cx, self.cy):
            return None
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y, z

    # ── YOLO mask → detections ───────────────────────────────
    def extract_detections(self, result, image_h, image_w):
        detections = []
        if result.masks is None or result.masks.data is None:
            return detections
        masks = result.masks.data.detach().cpu().numpy()
        boxes = result.boxes
        confs = clss = None
        if boxes is not None:
            if boxes.conf is not None:
                confs = boxes.conf.detach().cpu().numpy()
            if boxes.cls is not None:
                clss = boxes.cls.detach().cpu().numpy()

        for i, mask in enumerate(masks):
            if mask.shape[:2] != (image_h, image_w):
                mask = cv2.resize(
                    mask, (image_w, image_h), interpolation=cv2.INTER_NEAREST)
            binary = (mask > 0.5).astype(np.uint8) * 255
            contours, _ = cv2.findContours(
                binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                continue
            contour = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(contour))
            if area < self.min_mask_area:
                continue
            M = cv2.moments(contour)
            if abs(M["m00"]) < 1e-6:
                continue
            cx = float(M["m10"] / M["m00"])
            cy = float(M["m01"] / M["m00"])
            conf = float(confs[i]) if confs is not None and i < len(confs) else 1.0
            cls_id = int(clss[i]) if clss is not None and i < len(clss) else -1
            detections.append({
                "contour": contour,
                "area": area,
                "center": np.array([cx, cy], dtype=np.float32),
                "conf": conf,
                "cls_name": self._class_id_to_name(cls_id),
            })
        return detections

    def _class_id_to_name(self, cls_id):
        if cls_id is None or cls_id < 0:
            return None
        names = getattr(self.model, "names", None)
        if names is None:
            return None
        if isinstance(names, dict):
            return names.get(cls_id)
        try:
            return names[cls_id]
        except (IndexError, KeyError, TypeError):
            return None

    # ── main callback ────────────────────────────────────────
    def image_callback(self, msg: Image):
        try:
            frame_bgr = imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"image conversion failed: {e}")
            return

        h, w = frame_bgr.shape[:2]
        debug = frame_bgr.copy()
        start = time.time()

        try:
            with torch.inference_mode():
                results = self.model.predict(
                    source=frame_bgr, imgsz=self.imgsz, conf=self.conf,
                    iou=self.iou, device=self.device, half=self.half,
                    verbose=False, retina_masks=True)
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        detections = self.extract_detections(results[0], h, w)
        targets = [d for d in detections
                   if not self.target_class_name
                   or d.get("cls_name") == self.target_class_name]

        published = False
        if targets:
            # 가장 큰(가까운) 컵 한 개 선택
            best = max(targets, key=lambda d: d["area"])
            u, v = best["center"]
            z = self.get_depth_at_pixel(u, v)
            if z is not None:
                p_cam = self.deproject_pixel_to_3d(u, v, z)
                if p_cam is not None:
                    self.publish_pose(p_cam, msg.header)
                    published = True
                    cv2.drawContours(debug, [best["contour"]], -1, (0, 200, 255), 2)
                    cv2.circle(debug, (int(u), int(v)), 5, (0, 0, 255), -1)
                    cv2.putText(
                        debug,
                        f"({p_cam[0]:+.3f},{p_cam[1]:+.3f},{p_cam[2]:.3f})m",
                        (int(u) + 8, int(v)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 1)

        # 비선택 타겟도 옅게 표시
        for d in targets:
            if not published or not np.array_equal(d["center"], best["center"]):
                cv2.drawContours(debug, [d["contour"]], -1, (120, 120, 120), 1)
        cv2.putText(
            debug, f"mouth-up cups={len(targets)} published={published}",
            (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        self.publish_debug(debug, msg.header)

        elapsed = (time.time() - start) * 1000.0
        self.get_logger().info(
            f"mouth-up cups={len(targets)} published={published} "
            f"time={elapsed:.1f} ms")

    def publish_pose(self, p_cam, header):
        ps = PoseStamped()
        ps.header.stamp = header.stamp
        ps.header.frame_id = (
            self.optical_frame or header.frame_id or "camera_color_optical_frame")
        ps.pose.position.x = float(p_cam[0])
        ps.pose.position.y = float(p_cam[1])
        ps.pose.position.z = float(p_cam[2])
        ps.pose.orientation.w = 1.0
        self.pose_pub.publish(ps)

    def publish_debug(self, image_bgr, header):
        try:
            out = cv2_to_imgmsg(image_bgr, encoding="bgr8")
            out.header = header
            self.debug_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"debug image publish failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MouthUpCupPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
