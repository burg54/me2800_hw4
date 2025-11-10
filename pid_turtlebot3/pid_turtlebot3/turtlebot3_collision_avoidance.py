#!/usr/bin/env python3

# Importing required modules
import rclpy  # ROS 2 Python client library
#import math  # For mathematical calculations
from rclpy.node import Node  # Base class for ROS 2 nodes
from geometry_msgs.msg import Twist  # Message type for velocity commands
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan # import sensor message package
#from gazebo_msgs.srv import SpawnEntity #GetModelList #GetModelState
#from cpp_node.action import GoToPose  # Custom action for GoToPose
#from rclpy.action import ActionServer, GoalResponse  # ROS 2 Action Server utilities
#from rclpy.action.server import ServerGoalHandle  # Goal handle for the action server
#from rclpy.callback_groups import ReentrantCallbackGroup  # To allow concurrent callbacks
#from rclpy.wait_for_message import wait_for_message  # asdf




# Define the main controller node class
class CA_Node(Node):
    def __init__(self):
        super().__init__('turtlebot3_collision_avoidance')  # Initialize the node with a name
        self.get_logger().info("Node Started")  # Log node initialization

        # Publisher for sending velocity commands
        self.my_vel_command = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber_ = self.create_subscription(LaserScan,'/scan',self.timer_callback, 10)


    # Callback to process turtle's laser scan and compute control commands
    def timer_callback(self, msg): #: LaserScan):
        # Publish feedback/scan for the action
        print('------------------')
        print('Front-direction laser scan:', msg.ranges[0])
        print('15 deg laser scan:', msg.ranges[15])
        print('Left-direction laser scan:', msg.ranges[90])
        print('Right-direction laser scan:', msg.ranges[270])
        print('345 deg laser scan:', msg.ranges[345])

        # check if there is no obstacle in front and to the front-left and front-right.....
        if msg.ranges[0] > 0.75 and msg.ranges[15] > 0.75 and msg.ranges[345] > 0.75:
            l_v = 0.5 #move forward in the x direction with a velocity of ...
            a_v = 0. #rotate in the z axis with a angular velocity of ...
        # else, if there's obstacle, do something else	
        else:
            l_v = 0.5 
            a_v = 0. 
   
        # Send computed velocity commands
        self.my_velocity_cont(l_v, a_v)

    # Publish velocity commands to the topic
    def my_velocity_cont(self, l_v, a_v):
        my_msg = Twist()
        my_msg.linear.x = l_v  # Linear velocity
        my_msg.angular.z = a_v  # Angular velocity
        self.my_vel_command.publish(my_msg)  # Publish the message




# Entry point of the script
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    node = CA_Node()  # Create an instance of the node
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()  # Destroy the node when done

# Run the script
if __name__ == '__main__':
    #if wait_for_gazebo_service():
        # Proceed with the rest of your robot control logic
    main()
    
