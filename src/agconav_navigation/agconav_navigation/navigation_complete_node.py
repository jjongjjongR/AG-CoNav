import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray, GoalStatus
from std_msgs.msg import Bool

class NavigationCompleteNode(Node):
    def __init__(self):
        super().__init__('navigation_complete_node')
        
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            'navigate_to_pose/_action/status',
            self.status_callback,
            10
        )
        from rclpy.qos import QoSProfile, DurabilityPolicy
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(Bool, 'navigation_status', qos_profile)

    def status_callback(self, msg: GoalStatusArray):
        if not msg.status_list:
            return

        # Check the latest status in the array
        latest_status = msg.status_list[-1].status
        
        status_msg = Bool()
        # STATUS_SUCCEEDED = 4
        if latest_status == GoalStatus.STATUS_SUCCEEDED:
            status_msg.data = True
        elif latest_status in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_ACCEPTED):
            status_msg.data = False
        else:
            status_msg.data = False

        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationCompleteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
