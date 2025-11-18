import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.time import Time


class DriveToCoord(Node):
    def __init__(self):
        super().__init__('drive_to_goal')
        self.get_logger().info('DriveToGoal init')

        self.last_BehaviorTreeNode = None
        self.last_btDriveCoord = None
        self.last_peoplesearchcoord = None

        self.currentgoal = None

        # Action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.status_pub = self.create_publisher(String, '/drive_to_coord_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # Subscribers
        self.BehaviorTreeNode_sub = self.create_subscription(String, '/BehaviorTreeNode', self.BehaviorTreeNode_callback, 10)
        self.btDriveCoord_sub = self.create_subscription(PoseStamped, '/btDriveCoord', self.btDriveCoord_callback, 10)
        self.BTnode_sub = self.create_subscription(PoseStamped, '/peoplesearchcoord', self.peoplesearchcoord_callback, 10)

        #wachten op nav server & Status versturen
        self._action_client.wait_for_server()
        self.get_logger().info('DriveToGoal node gestart.')
        self.publish_status(10, "gestart")


    def BehaviorTreeNode_callback(self, msg):
        self.get_logger().info("Nieuw topic ontvangen van BehaviorTree.")
        self.last_BehaviorTreeNode = msg.data.strip()

        self.actiondistribute()

    def btDriveCoord_callback(self, msg):
        self.last_btDriveCoord = msg
        self.publish_status(11, "coördinaat opgeslagen")
        self.get_logger().info('Nieuw coördinaat ontvangen.')

    def peoplesearchcoord_callback(self, msg):
        self.last_peoplesearchcoord = msg

        self.actiondistribute()

    def actiondistribute(self):
        if self.last_BehaviorTreeNode in ["IsRobotAtQuiz"]:
            if self.last_btDriveCoord is None:
                self.get_logger().warn("Geen coördinaat ontvangen — goal niet gestuurd!")
                self.publish_status(12, "geen coördinaat")
                return
            self.last_btDriveCoord.header.stamp = Time().to_msg()
            self.send_goal(self.last_btDriveCoord)

        elif self.last_BehaviorTreeNode in ["StartDrivingToPeople", "CheckingNearbyVisitors"]:
            if self.last_peoplesearchcoord is None:
                self.get_logger().warn("Geen coördinaat ontvangen — goal niet gestuurd!")
                self.publish_status(12, "geen coördinaat")
                return
            self.send_goal(self.last_peoplesearchcoord)

        elif self.last_BehaviorTreeNode in ["DriveToChargingStation","StatusDriveToChargingDock","IsRobotCharging","IsBatteryFull","BatteryCharged"]:
            return
            #Navigatie laten rusten bij batterijtopics
        else:
            self.emergencystop()
            self.get_logger().warn("Niet relevant voor drive_to_goal.")

    def emergencystop(self):
        self._goal_handle.cancel_goal_async()
        for i in range(20):
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            Time.sleep(0.1)

    def send_goal(self, coordinate):
        self.currentgoal = NavigateToPose.Goal()
        self.currentgoal.pose = coordinate

        self.publish_status(13, "goal verzonden")

        self._send_goal_future = self._action_client.send_goal_async(self.currentgoal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status(10, "goal afgewezen")
            self.get_logger().warn("Goal NIET geaccepteerd door Nav2!")
            return

        self.get_logger().info("Goal geaccepteerd ✅")
        self.publish_status(15, "goal geaccepteerd")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        self.publish_status(status, "NAV")  # NAV statussen
        self.get_logger().info(f"Goal afgerond, Nav2-status: {status:02d}")

    def publish_status(self, status, text: str):
        msg = String()

        if self.last_btDriveCoord == None:
            stamp = "0000000000000000000"
        else:
            stamp = f"{self.last_btDriveCoord.header.stamp.sec:010d}.{self.last_btDriveCoord.header.stamp.nanosec:09d}"

        msg.data = f"{status:02d}" + "-" + stamp + "-" + text

        self.status_pub.publish(msg)
        self.get_logger().info(f"[STATUS] {text}")


def main(args=None):
    rclpy.init(args=args)
    node = DriveToCoord()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Afgebroken door gebruiker.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

