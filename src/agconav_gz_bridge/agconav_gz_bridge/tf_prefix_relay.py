"""TF prefix 리레이 — 한 로봇의 사설 TF를 전역 /tf로 접두어 붙여 재발행한다.

CHAMP나 clearpath처럼 frame 이름을 하드코딩해 frame_prefix를 못 쓰는 스택을
멀티로봇에서 격리하기 위한 노드.

동작:
  - 로봇 스택은 자기 노드들의 /tf, /tf_static 를 사설 토픽(예: /leg/tf)으로 remap해서 쓴다.
    (그 안에서는 base_link 등 루트 프레임 그대로라 CHAMP/clearpath가 정상 동작)
  - 이 노드가 사설 TF를 구독해, 공유 프레임(map 등)만 빼고 모든 frame_id/child_frame_id에
    "<prefix>/"를 붙여 전역 /tf 로 재발행한다. → 외부 모듈은 leg/base_link 등 계약 프레임을 본다.

파라미터:
  prefix          : 붙일 접두어 (예: "leg")
  input_tf        : 사설 동적 TF 토픽 (기본 "tf" → 노드 네임스페이스 하위)
  input_tf_static : 사설 정적 TF 토픽 (기본 "tf_static")
  shared_frames   : 접두어를 붙이지 않을 공유 프레임 목록 (기본 ["map"])
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from tf2_msgs.msg import TFMessage


class TfPrefixRelay(Node):
    def __init__(self):
        super().__init__('tf_prefix_relay')
        self.declare_parameter('prefix', '')
        self.declare_parameter('input_tf', 'tf')
        self.declare_parameter('input_tf_static', 'tf_static')
        self.declare_parameter('shared_frames', ['map'])

        prefix = self.get_parameter('prefix').value
        in_tf = self.get_parameter('input_tf').value
        in_tf_static = self.get_parameter('input_tf_static').value
        self._shared = set(self.get_parameter('shared_frames').value)
        if not prefix:
            raise RuntimeError('prefix 파라미터가 필요합니다.')
        self._prefix = prefix.rstrip('/') + '/'

        # 정적 TF는 transient_local(래치)로 주고받아야 늦게 붙는 구독자도 받는다.
        static_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self._pub_tf = self.create_publisher(TFMessage, '/tf', 10)
        self._pub_tf_static = self.create_publisher(TFMessage, '/tf_static', static_qos)

        # static TF는 여러 퍼블리셔가 각각 래치한다. 리레이가 그대로 재발행하면 depth=1
        # 구독자는 마지막 것만 받는다. 그래서 본 것들을 누적해 매번 "전체 집합"을 재발행한다
        # (tf2 StaticTransformBroadcaster와 동일한 동작).
        self._static = {}  # child_frame_id -> TransformStamped

        self.create_subscription(TFMessage, in_tf, self._on_tf, 10)
        self.create_subscription(TFMessage, in_tf_static, self._on_tf_static, static_qos)
        self.get_logger().info(
            f"tf_prefix_relay: '{in_tf}'/'{in_tf_static}' -> /tf(_static) "
            f"prefix='{self._prefix}' shared={sorted(self._shared)}")

    def _fix(self, frame):
        # 공유 프레임(map 등)과 이미 접두어가 붙은 프레임은 그대로 둔다.
        if frame in self._shared or frame.startswith(self._prefix):
            return frame
        return self._prefix + frame

    def _remap(self, msg):
        for t in msg.transforms:
            t.header.frame_id = self._fix(t.header.frame_id)
            t.child_frame_id = self._fix(t.child_frame_id)
        return msg

    def _on_tf(self, msg):
        self._pub_tf.publish(self._remap(msg))

    def _on_tf_static(self, msg):
        remapped = self._remap(msg)
        for t in remapped.transforms:
            self._static[t.child_frame_id] = t
        self._pub_tf_static.publish(TFMessage(transforms=list(self._static.values())))


def main(args=None):
    rclpy.init(args=args)
    node = TfPrefixRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
