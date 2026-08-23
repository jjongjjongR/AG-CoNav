#!/usr/bin/env python3
"""Finish a partially completed Nav2 lifecycle bringup.

Large static maps can keep one lifecycle service busy beyond Nav2's internal
timeout.  The expensive allocation is nevertheless complete afterwards, so a
second, ordered pass can safely activate only the nodes left behind.
"""
import sys

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


NODE_ORDER = (
    'controller_server', 'smoother_server', 'planner_server', 'route_server',
    'behavior_server', 'velocity_smoother', 'collision_monitor',
    'bt_navigator', 'waypoint_follower', 'docking_server',
)


class Nav2LifecycleRecover(Node):
    def __init__(self):
        super().__init__('nav2_lifecycle_recover')
        self.declare_parameter('target_namespace', 'wheel')
        self.ns = self.get_parameter('target_namespace').value.strip('/')

    def _call(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=10.0):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def state(self, name):
        client = self.create_client(GetState, f'/{self.ns}/{name}/get_state')
        response = self._call(client, GetState.Request(), 15.0)
        return response.current_state.id if response else None

    def transition(self, name, transition_id):
        client = self.create_client(ChangeState, f'/{self.ns}/{name}/change_state')
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = self._call(client, request, 180.0)
        return bool(response and response.success)

    def run(self):
        failures = []
        for name in NODE_ORDER:
            state = self.state(name)
            if state == State.PRIMARY_STATE_ACTIVE:
                continue
            if state == State.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().info(f'{self.ns}/{name}: configure 재시도')
                self.transition(name, Transition.TRANSITION_CONFIGURE)
                state = self.state(name)
            if state == State.PRIMARY_STATE_INACTIVE:
                self.get_logger().info(f'{self.ns}/{name}: activate 재시도')
                self.transition(name, Transition.TRANSITION_ACTIVATE)
                state = self.state(name)
            if state != State.PRIMARY_STATE_ACTIVE:
                failures.append(name)
        if failures:
            self.get_logger().error(
                f'{self.ns} Nav2 복구 미완료: {", ".join(failures)}')
            return False
        self.get_logger().info(f'{self.ns} Nav2 전체 active 확인')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = Nav2LifecycleRecover()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        sys.exit(1)


if __name__ == '__main__':
    main()
