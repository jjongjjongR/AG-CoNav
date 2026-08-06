#!/usr/bin/env python3
"""
drone_path_player

역할 (README 3.1, 4장 모듈 A):
    - path.yaml(waypoint 목록)을 읽어 /drone/cmd_pose (PoseStamped)를 주기적으로 발행
    - 드론은 kinematic이므로 map -> drone/base_link TF를 이 노드가 직접 발행
      (odom, EKF 없음 - README 3.1 "드론 TF" 확정 사항)

주의 (팀 확인 필요):
    README 확정 파라미터 표에는 이동 속도(m/s)가 없습니다.
    waypoint 사이를 시간 기준으로 보간(interpolate)하려면 속도가 필요해서
    cruise_speed_mps 파라미터를 임시로 추가했습니다. 팀에서 값 확정 부탁드립니다.
"""

import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def slerp(q0, q1, t):
    """두 quaternion(dict: x,y,z,w) 사이를 구면 선형보간"""
    x0, y0, z0, w0 = q0["x"], q0["y"], q0["z"], q0["w"]
    x1, y1, z1, w1 = q1["x"], q1["y"], q1["z"], q1["w"]

    dot = x0 * x1 + y0 * y1 + z0 * z1 + w0 * w1
    if dot < 0.0:
        x1, y1, z1, w1 = -x1, -y1, -z1, -w1
        dot = -dot

    if dot > 0.9995:
        # 각도가 거의 같으면 선형보간 후 정규화
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        z = z0 + t * (z1 - z0)
        w = w0 + t * (w1 - w0)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        return {"x": x / norm, "y": y / norm, "z": z / norm, "w": w / norm}

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_theta = math.sin(theta)

    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0

    return {
        "x": s0 * x0 + s1 * x1,
        "y": s0 * y0 + s1 * y1,
        "z": s0 * z0 + s1 * z1,
        "w": s0 * w0 + s1 * w1,
    }


def lerp_position(p0, p1, t):
    return {
        "x": p0["x"] + t * (p1["x"] - p0["x"]),
        "y": p0["y"] + t * (p1["y"] - p0["y"]),
        "z": p0["z"] + t * (p1["z"] - p0["z"]),
    }


def distance(p0, p1):
    return math.sqrt(
        (p1["x"] - p0["x"]) ** 2
        + (p1["y"] - p0["y"]) ** 2
        + (p1["z"] - p0["z"]) ** 2
    )


class DronePathPlayer(Node):
    def __init__(self):
        super().__init__("drone_path_player")

        # ===== 파라미터 (README 확정 표 기준) =====
        self.declare_parameter("path_file", "")            # 필수
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("loop", False)               # 지역 1회 스캔이므로 기본 false
        self.declare_parameter("interpolate", True)
        # use_sim_time은 ROS2가 모든 노드에 자동으로 선언하는 built-in 파라미터라
        # 여기서 다시 declare_parameter 하면 ParameterAlreadyDeclaredException 발생.
        # 필요하면 self.get_parameter("use_sim_time").value 로 값만 읽으면 됨.
        # README에 없는 값 - 팀 확정 필요. 업계 표준(라이다 매핑 미션 기준
        # 5-10 m/s: UgCS 기본 5 m/s/DJI 센서 최대 8-9 m/s, Propeller Aero
        # DJI L1/L2 9 m/s 이하 권장, Anvil Labs DJI M300 5-10 m/s)을 참고해
        # 3.0 -> 8.0으로 조정. 최종 확정은 팀 논의 필요.
        self.declare_parameter("cruise_speed_mps", 8.0)

        self.path_file = self.get_parameter("path_file").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.loop = self.get_parameter("loop").value
        self.interpolate = self.get_parameter("interpolate").value
        self.cruise_speed = self.get_parameter("cruise_speed_mps").value

        if not self.path_file:
            self.get_logger().error("path_file 파라미터가 비어 있습니다. 종료합니다.")
            raise SystemExit(1)

        self.waypoints = self._load_waypoints(self.path_file)
        if len(self.waypoints) < 1:
            self.get_logger().error("path_file에 waypoint가 없습니다. 종료합니다.")
            raise SystemExit(1)

        self.get_logger().info(
            f"waypoint {len(self.waypoints)}개 로드 완료 (path_file={self.path_file})"
        )

        # QoS: 명령(cmd_pose)은 reliable - README 3.4
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pose_pub = self.create_publisher(PoseStamped, "/drone/cmd_pose", qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ground_elevation_mapper의 navigation_status(Bool, latched)와 동일한 패턴:
        # 경로 재생이 끝났음을 다른 노드(예: drone_elevation_mapper)가 늦게
        # 구독해도 놓치지 않도록 latched로 한 번만 발행한다.
        path_status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.path_status_pub = self.create_publisher(
            Bool, "/drone/path_status", path_status_qos
        )

        # 재생 상태
        self.current_idx = 0
        self.segment_elapsed = 0.0
        self.finished = False

        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self._on_timer)

    def _load_waypoints(self, path_file):
        with open(path_file, "r") as f:
            data = yaml.safe_load(f)
        return data.get("waypoints", [])

    def _on_timer(self):
        if self.finished:
            return

        dt = 1.0 / self.publish_rate_hz

        # waypoint가 1개뿐이면 그대로 계속 발행하고 종료 처리
        if len(self.waypoints) == 1:
            self._publish_pose(self.waypoints[0])
            self._finish()
            return

        wp_a = self.waypoints[self.current_idx]
        wp_b = self.waypoints[self.current_idx + 1]

        if self.interpolate:
            seg_len = distance(wp_a["position"], wp_b["position"])
            seg_duration = max(seg_len / self.cruise_speed, 1e-3)
            t = min(self.segment_elapsed / seg_duration, 1.0)

            pos = lerp_position(wp_a["position"], wp_b["position"], t)
            ori = slerp(wp_a["orientation"], wp_b["orientation"], t)
            self._publish_pose({"position": pos, "orientation": ori})

            self.segment_elapsed += dt
            if t >= 1.0:
                self._advance_waypoint()
        else:
            # 보간 없이 waypoint를 순서대로 그대로 발행
            self._publish_pose(wp_a)
            self._advance_waypoint()

    def _advance_waypoint(self):
        self.segment_elapsed = 0.0
        self.current_idx += 1

        if self.current_idx >= len(self.waypoints) - 1:
            if self.loop:
                self.current_idx = 0
            else:
                self._finish()

    def _finish(self):
        self.finished = True
        self.get_logger().info("경로 재생 종료 (waypoint 남지 않음, 스캔 완료)")
        self.path_status_pub.publish(Bool(data=True))

    def _publish_pose(self, wp):
        now = self.get_clock().now().to_msg()

        msg = PoseStamped()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(wp["position"]["x"])
        msg.pose.position.y = float(wp["position"]["y"])
        msg.pose.position.z = float(wp["position"]["z"])
        msg.pose.orientation.x = float(wp["orientation"]["x"])
        msg.pose.orientation.y = float(wp["orientation"]["y"])
        msg.pose.orientation.z = float(wp["orientation"]["z"])
        msg.pose.orientation.w = float(wp["orientation"]["w"])
        self.pose_pub.publish(msg)

        # README 3.1: 드론은 kinematic -> 이 노드가 map->drone/base_link TF 직접 발행
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.frame_id
        tf_msg.child_frame_id = "drone/base_link"
        tf_msg.transform.translation.x = msg.pose.position.x
        tf_msg.transform.translation.y = msg.pose.position.y
        tf_msg.transform.translation.z = msg.pose.position.z
        tf_msg.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DronePathPlayer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
