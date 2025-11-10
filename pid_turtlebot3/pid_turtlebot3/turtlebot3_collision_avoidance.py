#!/usr/bin/env python3

# Importing required modules
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from geometry_msgs.msg import Twist  # Message type for velocity commands
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan # import sensor message package


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

        # write some code to check if there is no obstacle in front and to the front-left and front-right.....
        # hints are given with the print commands above
        # a laser scan return at angle 0 indicates something is in front of the robot
        # a laser scan return at angle 15,345 indicate something is slightly to the left (right) of the robot
        # something like a reading of 0.75 indicates that a collision will occur soon
        # can write some logic like ''if range to front is less than a certain amount
        #                              and range to left is less than a certain amount, and range to right...''
        # then issue a turning command
        # otherwise drive forward!
        # this will be in terms of linear and angular velocity, l_v and a_v, which get sent below
        # recall python syntax for multiple conditions, '
        #    if X and Y and Z and A:
        #        something
        #        something else
        #    elif A or B:
        #        something else entirely
        #    else:
        #        all the things   
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
    
