#!/usr/bin/env python3
"""
drone_pose_controller

역할 (README 4장 모듈 A, drone_path_player와 짝을 이루는 노드):
    /drone/cmd_pose(PoseStamped)를 구독해 ros_gz_interfaces/srv/SetEntityPose로
    /world/<world_name>/set_pose를 호출, Gazebo X3 모델의 실제 pose를 그 값으로 바꾼다.
    드론은 kinematic이라 물리 시뮬레이션이 아니라 world 좌표를 직접 설정한다
    (drone_path_player와 동일한 전제 - README 3.1).

전제:
    allowed_frame(map)과 Gazebo world 원점이 일치하므로 좌표 변환은 하지 않는다.

서비스 호출 실패 처리 (팀 플로우차트):
    성공 -> 그대로 다음 명령 대기.
    실패 -> /status에 실패 로그 발행 + 재시도. max_retries 초과 시 중단하고
            에러 로그를 명확히 남긴 뒤, 노드 자체는 죽지 않고 다음 cmd_pose를 계속 기다린다
            (한 번의 실패로 컨트롤러 전체가 멈추면 안 되므로).

구현 메모 (중요):
    rclpy의 Client.call()은 "콜백 안에서 부르면 데드락 위험이 있다"는 경고가 있다
    (SingleThreadedExecutor가 구독 콜백을 처리 중인 스레드에 묶여 있으면, 같은 스레드가
    서비스 응답을 기다리며 블로킹하는 동안 그 응답을 처리할 스레드가 없어서 멈춘다).
    그래서 구독 콜백(_on_cmd_pose)은 목표 pose를 큐에 넣기만 하고, 실제 서비스 호출과
    재시도는 별도 워커 스레드(_worker_loop)에서 처리한다. 큐 깊이는 1로 유지해서,
    재시도 도중 더 최신 목표가 들어오면 오래된 목표는 버리고 최신 목표로 갈아탄다
    (뒤처진 목표를 뒤늦게 쫓아가지 않기 위해 - drone_path_player가 10Hz로 촘촘하게
    보간된 pose를 계속 보내주는 구조라, 오래된 목표를 큐잉하면 계속 밀리기만 한다).
"""

import queue
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


class DronePoseController(Node):
    def __init__(self):
        super().__init__("drone_pose_controller")

        # ===== 파라미터 =====
        self.declare_parameter("world_name", "")          # 필수 - 기본값 없음
        self.declare_parameter("entity_name", "X3")        # Seongdong_gu.world의 X3 <name>
        self.declare_parameter("allowed_frame", "map")
        self.declare_parameter("service_timeout_sec", 1.0)
        self.declare_parameter("max_retries", 3)
        # use_sim_time은 ROS2가 모든 노드에 자동으로 선언하는 built-in 파라미터.

        self.world_name = self.get_parameter("world_name").value
        self.entity_name = self.get_parameter("entity_name").value
        self.allowed_frame = self.get_parameter("allowed_frame").value
        self.service_timeout_sec = self.get_parameter("service_timeout_sec").value
        self.max_retries = self.get_parameter("max_retries").value

        if not self.world_name:
            self.get_logger().error("world_name 파라미터가 비어 있습니다. 종료합니다.")
            raise SystemExit(1)

        service_name = f"/world/{self.world_name}/set_pose"
        self.client = self.create_client(SetEntityPose, service_name)

        self.status_pub = self.create_publisher(String, "/status", 10)

        # QoS: cmd_pose는 drone_path_player와 동일하게 reliable - README 3.4
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/drone/cmd_pose", self._on_cmd_pose, qos
        )

        self._queue = queue.Queue(maxsize=1)
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"drone_pose_controller 시작: service={service_name}, "
            f"entity={self.entity_name}, max_retries={self.max_retries}"
        )

    def _on_cmd_pose(self, msg: PoseStamped):
        if msg.header.frame_id != self.allowed_frame:
            self.get_logger().warn(
                f"허용되지 않은 frame_id='{msg.header.frame_id}' "
                f"(allowed_frame='{self.allowed_frame}') - 이 목표는 무시"
            )
            return

        if self._queue.full():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
        self._queue.put_nowait(msg.pose)

    def _worker_loop(self):
        while rclpy.ok():
            try:
                pose = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            self._drive_to_pose(pose)

    def _drive_to_pose(self, pose):
        request = SetEntityPose.Request()
        request.entity.name = self.entity_name
        request.entity.type = Entity.MODEL
        request.pose = pose

        for attempt in range(1, self.max_retries + 1):
            if not self._queue.empty():
                # 재시도하는 동안 더 최신 목표가 도착 - 이 목표는 포기하고 최신 것으로 넘어간다.
                self.get_logger().info("재시도 중 더 최신 목표 pose 도착 - 갈아탐")
                return

            if not self.client.wait_for_service(timeout_sec=self.service_timeout_sec):
                self._on_attempt_failure(attempt, "서비스를 사용할 수 없음 (대기 시간 초과)")
                continue

            response = self.client.call(request, timeout_sec=self.service_timeout_sec)

            if response is None:
                self._on_attempt_failure(attempt, "서비스 응답 시간 초과")
                continue
            if not response.success:
                self._on_attempt_failure(attempt, "서비스가 실패를 반환함 (success=False)")
                continue

            self.get_logger().debug(f"pose 변경 성공 (attempt={attempt})")
            return

        self.get_logger().error(
            f"pose 변경 실패: 최대 재시도 횟수({self.max_retries}) 초과, 중단 "
            f"(entity={self.entity_name}) - 다음 cmd_pose를 계속 기다립니다"
        )
        self._publish_status(
            f"pose 변경 실패: 최대 재시도 횟수({self.max_retries}) 초과, 중단"
        )

    def _on_attempt_failure(self, attempt, reason):
        text = f"pose 변경 실패 ({attempt}/{self.max_retries}): {reason}"
        self.get_logger().warn(text)
        self._publish_status(text)

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DronePoseController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
