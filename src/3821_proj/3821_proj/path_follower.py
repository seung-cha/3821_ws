#!/bin/use/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time


from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseArray     # Required by /path_points
from geometry_msgs.msg import Pose

from geometry_msgs.msg import Twist         # Required by /cmd_vel
from nav_msgs.msg import Odometry

from tf_transformations import *


from std_msgs.msg import Header


from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupTransform
from geometry_msgs.msg import TransformStamped      # Result of lookup transform

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

import math
import numpy

VELOCITY = 0.2
ROTATION_VELOCITY = 0.2
ROTATION_TOLERANCE = 0.15   # Robot is considered facing at the point if the angular difference is less than this.
GOAL_TOLERANCE = 0.15       # Robot is considered at the goal if the distance is less than this.

def cross_prod(a, b):
    result = [a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]]
    return result


class PointPublisher(Node):
    """
    Draws debugging line (path) in rviz
    """

    def __init__(self):
        super().__init__('node_3821_follower_debugger')
        self._pub = self.create_publisher(Marker, 'current_goal_point', 10)
        self._marker = Marker()

        self._marker.header = Header()
        self._marker.header.frame_id = 'map'
        self._marker.header.stamp = self.get_clock().now().to_msg()

        self._marker.ns = '3821_debug'
        self._marker.id = 0
        self._marker.type = Marker.CUBE
        self._marker.action = Marker.ADD

        self._marker.color = ColorRGBA()
        self._marker.color.a = 1.0
        self._marker.color.b = 1.0

        self._marker.scale.x = 0.05
        self._marker.scale.y = 0.05
        self._marker.scale.z = 0.05



    def Publish(self, x, y):
        """
        Start drawing a new line, erasing the previous line.
        """
        self._marker.action = Marker.DELETEALL
        self._pub.publish(self._marker)

        self._marker.pose.position.x = x
        self._marker.pose.position.y = y
        self._marker.action = Marker.ADD
        self._pub.publish(self._marker)





class RosNode(Node):

    def __init__(self):
        super().__init__('node_3821_follower')

        self.pathSub = self.create_subscription(PoseArray, '/path_points', self.OnPathPub, 10)
        self.odomSub = self.create_subscription(Odometry, '/odom', self.OnOdomPub, 10)
        self.cmdPub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.pointPublisher = PointPublisher()

        self.position = []
        self.rotation = float()

        self.path = []

        self.goal:Pose
        self.goal = None

        self.header: Header
        self.header = None

        self.create_timer(0.01, self.MoveRobot)

        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        

    
    def OnOdomPub(self, data:Odometry):
        pass

        # self.header = data.header
        # self.position = data.pose.pose.position

        # ori = data.pose.pose.orientation
        # quat = [ori.x, ori.y, ori.z, ori.w]
        # (r,p,y) = euler_from_quaternion(quat)
        # self.rotation = y


        #Problem: When rotating clickwise, the value changes from +0 to -0.
        #When rotating anti-clockwise, the value changes from -2pi to 2pi.
        #self.rotation = 2 * math.atan2(data.pose.pose.orientation.z, data.pose.pose.orientation.w)

        #print(f'Rotation: {self.rotation}')


    def OnPathPub(self, data:PoseArray):
        self.path = data.poses

    
    def GetNextGoal(self):
        self.goal = self.path.pop(0)
        self.pointPublisher.Publish(self.goal.position.x, self.goal.position.y)


    def Distance(self):

        # Convert from base_link to map
        tf:TransformStamped
    

        tf = self.tfBuffer.lookup_transform('map', 'base_link', Time(), Duration(seconds=1))


        r = [tf.transform.translation.x, tf.transform.translation.y]
        r1 = [self.goal.position.x, self.goal.position.y]        
        return math.dist(r, r1)


    def AngularDifference(self):
        
        tf:TransformStamped
        tf = self.tfBuffer.lookup_transform('base_link', 'map', Time(), Duration(seconds=1))

        
        transformedPoint = do_transform_pose(self.goal, tf)

        #print(f'transformed: {transformedPoint.position}')

        p = [transformedPoint.position.x, transformedPoint.position.y, 0.0]
        forward = [1.0, 0.0, 0.0]

        cross = numpy.cross(p, forward)
        l = math.dist(cross, [0.0, 0.0, 0.0])
        a = math.dist(p, [0.0, 0.0, 0.0])

        # Get the sign
        s = 1.0 if p[0] >= 0.0 else -1.0

        return s * math.asin(l / a)

        #return math.atan2(p[1] - tf.transform.translation.y, p[0] - tf.transform.translation.x) - self.rotation

    def GetDir(self):
        """The direction the robot should be rotating to"""
        tf:TransformStamped
        tf = self.tfBuffer.lookup_transform('base_link', 'map', Time(), Duration(seconds=1))

        transformedPoint = do_transform_pose(self.goal, tf)

        if transformedPoint.position.y >= 0.0:
            return 1.0
        else:
            return -1.0


    def MoveRobot(self):

        # Don't do anything if we don't have a goal
        if len(self.path) <= 0:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmdPub.publish(msg)
            return
        
        if self.goal is None:
            self.GetNextGoal()


        if self.Distance() < GOAL_TOLERANCE:
            self.goal = None    # Get the next goal
        else:
            # Drive the robot
            msg = Twist()
            angDiff = self.AngularDifference()
            if angDiff < ROTATION_TOLERANCE and angDiff >= 0.0:
                msg.linear.x = min(VELOCITY, VELOCITY * self.Distance())
            else: 
                msg.linear.x = 0.0
                msg.angular.z = self.GetDir() * min(ROTATION_VELOCITY, ROTATION_VELOCITY * abs(angDiff))

            
            self.cmdPub.publish(msg)

            print(f'Distance: {self.Distance()}')
            print(f'Difference: {self.AngularDifference()}')
            #print(f'speed: {msg.linear}')
            #print(f'angular: {msg.angular}')
            print('\n')


        





def main(args=None):
    rclpy.init(args=args)

    node = RosNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()