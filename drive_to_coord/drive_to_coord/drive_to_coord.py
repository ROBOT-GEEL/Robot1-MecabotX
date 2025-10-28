import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.time import Time


class drive_to_coord(Node):
	def __init__(self):
		super().__init__('drive_to_goal')

		self.lastcoord = None
		self._goal_handle = None

		self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

		self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

		self.BTnode_sub = self.create_subscription(
			String,
			'/BehaviorTreeNode',
			self.handle_BTnode_callback,
			10
		)

		self.coord_sub = self.create_subscription(
			PoseStamped,
			'/btDriveCoord',
			self.incomming_goal_callback,
			10
		)

		self.get_logger().info('DriveToGoal node gestart.')

	def incomming_goal_callback(self, msg):
		self.get_logger().info('coordinaat opgeslagen')
		self.lastcoord = msg

	def handle_BTnode_callback(self, msg: String):
		self.get_logger().info("nieuw topic ontvangen")
		data = msg.data.strip()
		if data in ["IsRobotAtQuiz", "drive"]:
			self.send_goal()
		else:
			self.get_logger().warn("Not a drive topic")

	def send_goal(self):

		if self.lastcoord is None:
			self.get_logger().warn("Geen coördinaat ontvangen, goal niet gestuurd!")
			return

		self.get_logger().info("send goal ontvangen")

		self._action_client.wait_for_server()
		self.get_logger().info("nav klaar")

		# Pose goed instellen
		goal_pose = PoseStamped()
		goal_pose.header.frame_id = 'base_link'
		goal_pose.header.stamp = Time().to_msg()  # nu = 0 → laatste TF
		goal_pose.pose = self.lastcoord.pose
		
		goal_pose.pose.position.x = 1.0  # 1 meter vooruit
		goal_pose.pose.position.y = 0.0
		goal_pose.pose.orientation.w = 1.0  # kijk dezelfde richting als robot

		goal = NavigateToPose.Goal()
		goal.pose = goal_pose

		# >>> HIER: werkelijk verzenden <<<
		self._send_goal_future = self._action_client.send_goal_async(goal)
		self._send_goal_future.add_done_callback(self.goal_response_callback)
		self.get_logger().info("goal verstuurd naar Nav2, wacht op accept")
		

	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().warn("Goal NIET geaccepteerd door Nav2!")
			return

		self.get_logger().info("Goal geaccepteerd ✅")
		self._goal_handle = goal_handle  # zodat cancel later kan

		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.result_callback)

	def result_callback(self, future):
		result = future.result().status
		self.get_logger().info(f"Goal afgewerkt, status: {result}")



def main(args=None):
	rclpy.init(args=args)
	drive_to_coord_inst = drive_to_coord()

	try:
		rclpy.spin(drive_to_coord_inst)

	except KeyboardInterrupt:
		drive_to_coord_inst.get_logger().info('Afgebroken door gebruiker.')

	finally:
		rclpy.shutdown()


if __name__ == '__main__':
	main()

