import json
import sqlite3
import sys
import types
from pathlib import Path


def _install_ros_stubs() -> None:
    if 'rclpy' in sys.modules:
        return

    rclpy_module = types.ModuleType('rclpy')
    rclpy_node_module = types.ModuleType('rclpy.node')

    class Node:
        pass

    rclpy_node_module.Node = Node
    rclpy_module.node = rclpy_node_module

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
    geometry_msgs_msg.PoseStamped = type('PoseStamped', (), {})
    geometry_msgs.msg = geometry_msgs_msg

    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msgs_msg.Image = type('Image', (), {})
    sensor_msgs.msg = sensor_msgs_msg

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')

    class String:
        def __init__(self) -> None:
            self.data = ''

    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg

    sys.modules['rclpy'] = rclpy_module
    sys.modules['rclpy.node'] = rclpy_node_module
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = geometry_msgs_msg
    sys.modules['sensor_msgs'] = sensor_msgs
    sys.modules['sensor_msgs.msg'] = sensor_msgs_msg
    sys.modules['std_msgs'] = std_msgs
    sys.modules['std_msgs.msg'] = std_msgs_msg

    numpy_module = types.ModuleType('numpy')
    numpy_module.uint8 = int
    numpy_module.frombuffer = lambda data, dtype=None: data
    sys.modules['numpy'] = numpy_module


_install_ros_stubs()
sys.path.append('ros2_ws/src/infa_inference')

from infa_inference.defect_inference_node import DefectInferenceNode


class _FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _Stamp:
    def __init__(self, sec: int, nanosec: int) -> None:
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self, sec: int, nanosec: int) -> None:
        self.stamp = _Stamp(sec, nanosec)


class _Image:
    def __init__(self, sec: int = 1, nanosec: int = 0) -> None:
        self.header = _Header(sec, nanosec)
        self.data = b'\x00\x01\x02\x03'
        self.height = 1
        self.width = 1
        self.encoding = 'rgb8'
        self.is_bigendian = 0
        self.step = 4


def _build_node(tmp_path: Path):
    node = DefectInferenceNode.__new__(DefectInferenceNode)
    node._model = object()
    node._confidence_threshold = 0.85
    node._debounce_iou = 0.5
    node._debounce_window_ns = int(2.5e9)
    node._latest_pose = (1.1, 2.2, 3.3)
    node._recent_records = []
    node._capture_dir = tmp_path / 'captures'
    node._capture_dir.mkdir(parents=True)
    node._jsonl_path = tmp_path / 'events.jsonl'
    node._conn = sqlite3.connect(tmp_path / 'events.sqlite3')
    node._init_db()
    node._event_pub = _FakePublisher()
    return node


def test_confidence_threshold_raw_capture_and_xyz_logging(tmp_path: Path):
    node = _build_node(tmp_path)
    node._infer_detections = lambda _msg: [
        {'class': 'structural_crack', 'confidence': 0.90, 'bbox': (0.0, 0.0, 10.0, 10.0)},
        {'class': 'surface_spall', 'confidence': 0.85, 'bbox': (20.0, 20.0, 30.0, 30.0)},
    ]

    node._on_image(_Image())

    rows = list(node._conn.execute('SELECT class_name, confidence, severity, x, y, z, image_path FROM defect_events'))
    assert len(rows) == 1
    class_name, confidence, severity, x, y, z, image_path = rows[0]

    assert class_name == 'structural_crack'
    assert confidence == 0.90
    assert severity == 'Critical'
    assert (x, y, z) == (1.1, 2.2, 3.3)
    assert Path(image_path).exists()

    meta_path = Path(image_path).with_suffix('.json')
    assert meta_path.exists()

    events = node._jsonl_path.read_text(encoding='utf-8').strip().splitlines()
    assert len(events) == 1
    payload = json.loads(events[0])
    assert payload['bbox'] == {'xmin': 0.0, 'ymin': 0.0, 'xmax': 10.0, 'ymax': 10.0}


def test_severity_mapping_and_deduplication_of_repeated_detections(tmp_path: Path):
    node = _build_node(tmp_path)
    node._infer_detections = lambda _msg: [
        {'class': 'corrosion_patch', 'confidence': 0.96, 'bbox': (1.0, 1.0, 9.0, 9.0)},
    ]

    node._on_image(_Image(sec=10, nanosec=0))
    node._on_image(_Image(sec=10, nanosec=100_000_000))

    rows = list(node._conn.execute('SELECT class_name, severity FROM defect_events'))
    assert rows == [('corrosion_patch', 'Major')]

    node._infer_detections = lambda _msg: [
        {'class': 'surface_stain', 'confidence': 0.97, 'bbox': (1.0, 1.0, 9.0, 9.0)},
    ]
    node._on_image(_Image(sec=13, nanosec=0))

    rows = list(node._conn.execute('SELECT class_name, severity FROM defect_events ORDER BY id'))
    assert rows == [('corrosion_patch', 'Major'), ('surface_stain', 'Minor')]
    assert len(node._event_pub.messages) == 2
