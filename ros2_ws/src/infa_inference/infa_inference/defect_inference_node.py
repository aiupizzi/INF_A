import json
import sqlite3
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - runtime dependency in deployment
    YOLO = None


SEVERITY_MAP = {
    'structural': 'Critical',
    'corrosion': 'Major',
    'surface': 'Minor',
}


@dataclass
class DetectionRecord:
    class_name: str
    confidence: float
    obb_aabb: Tuple[float, float, float, float]
    timestamp_ns: int


class DefectInferenceNode(Node):
    def __init__(self) -> None:
        super().__init__('infa_defect_inference_node')

        self.declare_parameter('image_topic', '/infa/camera/high_res/image_raw')
        self.declare_parameter('pose_topic', '/infa/local_pose')
        self.declare_parameter('event_topic', '/infa/inference/events')
        self.declare_parameter('model_path', 'models/yolov11_obb.pt')
        self.declare_parameter('confidence_threshold', 0.85)
        self.declare_parameter('debounce_iou_threshold', 0.5)
        self.declare_parameter('debounce_window_s', 2.5)
        self.declare_parameter('event_store_path', 'data/inference/events.sqlite3')
        self.declare_parameter('jsonl_store_path', 'data/inference/events.jsonl')
        self.declare_parameter('capture_dir', 'data/inference/raw_captures')

        self._confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self._debounce_iou = float(self.get_parameter('debounce_iou_threshold').value)
        self._debounce_window_ns = int(float(self.get_parameter('debounce_window_s').value) * 1e9)

        self._latest_pose: Optional[Tuple[float, float, float]] = None
        self._recent_records: List[DetectionRecord] = []

        self._capture_dir = Path(str(self.get_parameter('capture_dir').value))
        self._capture_dir.mkdir(parents=True, exist_ok=True)

        self._jsonl_path = Path(str(self.get_parameter('jsonl_store_path').value))
        self._jsonl_path.parent.mkdir(parents=True, exist_ok=True)

        sqlite_path = Path(str(self.get_parameter('event_store_path').value))
        sqlite_path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(sqlite_path)
        self._init_db()

        self._model = self._load_model(str(self.get_parameter('model_path').value))

        image_topic = str(self.get_parameter('image_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        event_topic = str(self.get_parameter('event_topic').value)

        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)
        self._event_pub = self.create_publisher(String, event_topic, 10)

        self.get_logger().info(f'Defect inference active. image_topic={image_topic}, pose_topic={pose_topic}')

    def destroy_node(self) -> bool:
        self._conn.close()
        return super().destroy_node()

    def _init_db(self) -> None:
        self._conn.execute(
            '''
            CREATE TABLE IF NOT EXISTS defect_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                timestamp_ns INTEGER NOT NULL,
                x REAL NOT NULL,
                y REAL NOT NULL,
                z REAL NOT NULL,
                class_name TEXT NOT NULL,
                confidence REAL NOT NULL,
                severity TEXT NOT NULL,
                image_path TEXT NOT NULL,
                bbox_xmin REAL NOT NULL,
                bbox_ymin REAL NOT NULL,
                bbox_xmax REAL NOT NULL,
                bbox_ymax REAL NOT NULL
            )
            '''
        )
        self._conn.commit()

    def _load_model(self, model_path: str) -> Optional[Any]:
        if YOLO is None:
            self.get_logger().error('ultralytics is not installed; inference disabled.')
            return None
        try:
            model = YOLO(model_path)
            self.get_logger().info(f'Loaded YOLO model from {model_path}')
            return model
        except Exception as exc:  # pragma: no cover - model/runtime dependent
            self.get_logger().error(f'Failed to load model {model_path}: {exc}')
            return None

    def _on_pose(self, msg: PoseStamped) -> None:
        self._latest_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _on_image(self, msg: Image) -> None:
        if self._model is None:
            return

        detections = self._infer_detections(msg)
        if not detections:
            return

        pose = self._latest_pose if self._latest_pose is not None else (0.0, 0.0, 0.0)
        stamp_ns = self._stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
        self._prune_recent(stamp_ns)

        for idx, detection in enumerate(detections):
            confidence = detection['confidence']
            if confidence <= self._confidence_threshold:
                continue

            class_name = detection['class']
            bbox = detection['bbox']

            if self._is_duplicate(class_name, bbox, stamp_ns):
                continue

            severity = self._class_to_severity(class_name)
            image_path = self._capture_raw_image(msg, stamp_ns, idx, class_name)
            timestamp_iso = datetime.fromtimestamp(stamp_ns / 1e9, tz=timezone.utc).isoformat()

            event = {
                'timestamp': timestamp_iso,
                'timestamp_ns': stamp_ns,
                'x': pose[0],
                'y': pose[1],
                'z': pose[2],
                'class': class_name,
                'confidence': confidence,
                'severity': severity,
                'image_path': image_path,
                'bbox': {
                    'xmin': bbox[0],
                    'ymin': bbox[1],
                    'xmax': bbox[2],
                    'ymax': bbox[3],
                },
            }

            self._persist_event(event)
            self._publish_event(event)
            self._recent_records.append(
                DetectionRecord(
                    class_name=class_name,
                    confidence=confidence,
                    obb_aabb=bbox,
                    timestamp_ns=stamp_ns,
                )
            )

    def _infer_detections(self, msg: Image) -> List[Dict[str, Any]]:
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8)
            channels = max(1, int(msg.step / msg.width)) if msg.width > 0 else 1
            if msg.height > 0 and msg.width > 0 and frame.size == msg.height * msg.width * channels:
                frame = frame.reshape((msg.height, msg.width, channels))
            results = self._model.predict(source=frame, verbose=False)
        except Exception as exc:  # pragma: no cover - model/runtime dependent
            self.get_logger().warning(f'Inference failed: {exc}')
            return []

        if not results:
            return []

        result = results[0]
        obb = getattr(result, 'obb', None)
        if obb is None:
            return []

        names = result.names if hasattr(result, 'names') else {}
        xyxyxyxy = obb.xyxyxyxy.tolist() if hasattr(obb, 'xyxyxyxy') else []
        confs = obb.conf.tolist() if hasattr(obb, 'conf') else []
        classes = obb.cls.tolist() if hasattr(obb, 'cls') else []

        detections: List[Dict[str, Any]] = []
        for points, conf, cls_idx in zip(xyxyxyxy, confs, classes):
            bbox = self._points_to_aabb(points)
            class_name = names.get(int(cls_idx), str(int(cls_idx))) if isinstance(names, dict) else str(int(cls_idx))
            detections.append({'class': class_name, 'confidence': float(conf), 'bbox': bbox})
        return detections

    def _points_to_aabb(self, points: List[List[float]]) -> Tuple[float, float, float, float]:
        xs = [float(p[0]) for p in points]
        ys = [float(p[1]) for p in points]
        return (min(xs), min(ys), max(xs), max(ys))

    def _stamp_to_ns(self, sec: int, nanosec: int) -> int:
        if sec == 0 and nanosec == 0:
            return self.get_clock().now().nanoseconds
        return sec * int(1e9) + nanosec

    def _class_to_severity(self, class_name: str) -> str:
        lowered = class_name.strip().lower()
        for key, severity in SEVERITY_MAP.items():
            if key in lowered:
                return severity
        return 'Minor'

    def _capture_raw_image(self, msg: Image, stamp_ns: int, det_idx: int, class_name: str) -> str:
        safe_class = ''.join(c.lower() if c.isalnum() else '_' for c in class_name).strip('_') or 'unknown'
        filename = f'{stamp_ns}_{safe_class}_{det_idx:02d}.raw'
        path = self._capture_dir / filename
        if not path.exists():
            path.write_bytes(msg.data)

        meta_filename = f'{stamp_ns}_{safe_class}_{det_idx:02d}.json'
        meta_path = self._capture_dir / meta_filename
        if not meta_path.exists():
            meta_path.write_text(
                json.dumps(
                    {
                        'height': msg.height,
                        'width': msg.width,
                        'encoding': msg.encoding,
                        'is_bigendian': msg.is_bigendian,
                        'step': msg.step,
                    },
                    indent=2,
                )
            )
        return str(path)

    def _is_duplicate(self, class_name: str, bbox: Tuple[float, float, float, float], now_ns: int) -> bool:
        for record in self._recent_records:
            if record.class_name != class_name:
                continue
            if now_ns - record.timestamp_ns > self._debounce_window_ns:
                continue
            if self._iou(record.obb_aabb, bbox) >= self._debounce_iou:
                return True
        return False

    def _prune_recent(self, now_ns: int) -> None:
        self._recent_records = [
            record for record in self._recent_records if now_ns - record.timestamp_ns <= self._debounce_window_ns
        ]

    def _iou(self, b1: Tuple[float, float, float, float], b2: Tuple[float, float, float, float]) -> float:
        x_left = max(b1[0], b2[0])
        y_top = max(b1[1], b2[1])
        x_right = min(b1[2], b2[2])
        y_bottom = min(b1[3], b2[3])

        inter_w = max(0.0, x_right - x_left)
        inter_h = max(0.0, y_bottom - y_top)
        inter_area = inter_w * inter_h

        area1 = max(0.0, b1[2] - b1[0]) * max(0.0, b1[3] - b1[1])
        area2 = max(0.0, b2[2] - b2[0]) * max(0.0, b2[3] - b2[1])
        union = area1 + area2 - inter_area

        if union <= 0.0:
            return 0.0
        return inter_area / union

    def _persist_event(self, event: Dict[str, Any]) -> None:
        self._conn.execute(
            '''
            INSERT INTO defect_events (
                timestamp, timestamp_ns, x, y, z, class_name, confidence, severity, image_path,
                bbox_xmin, bbox_ymin, bbox_xmax, bbox_ymax
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''',
            (
                event['timestamp'],
                event['timestamp_ns'],
                event['x'],
                event['y'],
                event['z'],
                event['class'],
                event['confidence'],
                event['severity'],
                event['image_path'],
                event['bbox']['xmin'],
                event['bbox']['ymin'],
                event['bbox']['xmax'],
                event['bbox']['ymax'],
            ),
        )
        self._conn.commit()

        with self._jsonl_path.open('a', encoding='utf-8') as fp:
            fp.write(json.dumps(event) + '\n')

    def _publish_event(self, event: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(event)
        self._event_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = DefectInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
