import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped		# message type van /btDriveCoord
from std_msgs.msg import String					# message type van /BehaviorTreeNode 
from geometry_msgs.msg import Twist  			# message type van /cmd_vel
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class drive_to_coord(Node):
	def __init__(self):
		super().__init__('drive_to_goal')

		self.lastcoord = None
		self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',1)
		# maak het mogelijk te publishen op het /cmd_vel-topic
		
		self.driveCoordStatus_pub = self.create_publisher(String, '/driveCoordStatus', 1)
		# maak het mogelijk te publishen op het /cmd_vel-topic
		
		self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')	
		# maak een client aan op de action-server 'navigate to pose
		
		# Maak een subscription aan op BTnode topic
		self.BTnode_sub = self.create_subscription(
			String,						# type van het bericht
			'/BehaviorTreeNode',			# topicnaam
			self.handle_BTnode_callback,		# callbackfunctie
			1								# queue size (aantal berichten bufferen)
		)
		
		# Maak een subscription aan op coordinate topic
		self.coord_sub = self.create_subscription(
			PoseStamped,					# type van het bericht
			'/btDriveCoord',				# topicnaam
			self.incomming_goal_callback, 	# callbackfunctie
			1								# queue size (aantal berichten bufferen)
		)
		
		self.get_logger().info('DriveToGoal node gestart.')
		
	def incomming_goal_callback(self, msg):
		self.lastcoord = msg
		
			
	def handle_BTnode_callback(self, msg: String):
		data = msg.data.strip()
		if data in ["btIsRobotAtQuiz", "drive"]:
			self.send_goal()
		else:
			self.emergency_stop()
	  	

	def send_goal(self):
	
		if self.lastcoord is None:
			self.driveCoordStatus("failure")
    		self.get_logger().warn("Geen coördinaat ontvangen, goal niet gestuurd!")
    		return
    
		# Wacht tot nav2 klaar is
		self._action_client.wait_for_server()
				
		#self.last_coord = NavigateToPose.Goal()
		#self.last_coord.pose.header.frame_id = 'map'
		#self.last_coord.pose.header.stamp = self.get_clock().now().to_msg()
		#self.last_coord.pose.pose.position.x = x
		#self.last_coord.pose.pose.position.y = y
		#self.last_coord.pose.pose.orientation.z = yaw  # eenvoudige yaw (niet correcte quaternion)
		
		goal = NavigateToPose.Goal()
		goal.pose = self.lastcoord
		
		self._send_goal_future = self._action_client.send_goal_async(goal)
		self._send_goal_future.add_done_callback(self.goal_response_callback)
		


	def goal_response_callback(self, future):
	
		if goal_handle.accepted:
			self.driveCoordStatus("rijden")
		else:
			self.driveCoordStatus("failure")

		goal_handle = future.result()

		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.result_callback)


	def result_callback(self, future):
		self.driveCoordStatus("success")
		
		
	def emergency_stop(self):
		# 1) annuleer navigatie
		if hasattr(self, '_goal_handle') and self._goal_handle is not None:
			self._goal_handle.cancel_goal_async()
			self.get_logger().warn("🚨 Navigatie geannuleerd!")

		# 2) stuur directe stop naar motorcontrollers
		stop_msg = Twist()
		stop_msg.linear.x = 0.0
		stop_msg.angular.z = 0.0
		self.cmd_vel_pub.publish(stop_msg)
		self.get_logger().warn("🛑 Robot onmiddellijk tot stilstand gebracht!")
		
		self.driveCoordStatus("failure")


	def driveCoordStatus(self, status: str):
		msg = String()
		msg.data = status
		self.driveCoordStatus_pub.publish(msg)
		self.get_logger().info(f"[driveCoordStatus] {status}")

		

def main(args=None):
	rclpy.init(args=args)
	drive_to_coord_inst = drive_to_coord()

	try:
		rclpy.spin(drive_to_coord_inst)

	except KeyboardInterrupt:
		drive_to_coord_inst.emergency_stop()
		drive_to_coord_inst.get_logger().info('Afgebroken door gebruiker.')
		
	finally:
		rclpy.shutdown()


if __name__ == '__main__':
	main()

