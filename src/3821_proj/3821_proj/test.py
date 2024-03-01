#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header

from nav_msgs.msg import Odometry                   # Required by /odom
from nav_msgs.msg import OccupancyGrid              # Required by /map
from geometry_msgs.msg import PointStamped          # Required by /clicked_point

from visualization_msgs.msg import MarkerArray      # Message to publish debugging message
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from geometry_msgs.msg import Point                 # Alternative data structure

# from tf2_ros.transform_listener import TransformListener
# from tf2_ros.buffer import Buffer
# from tf2_ros import LookupTransform
# from geometry_msgs.msg import TransformStamped

class PointI:
    """
    Accepts integers
    """

    def __init__(self, x = 0, y = 0, z = 0):
        self.x = x
        self.y = y
        self.z = z

class LineDrawer(Node):
    """
    Draws debugging line (path) in rviz
    """

    def __init__(self):
        super().__init__('node_3821_debugger')
        self._pub = self.create_publisher(Marker, 'line_drawer', 10)
        self._marker = Marker()

        self._marker.header = Header()
        self._marker.header.frame_id = 'map'
        self._marker.header.stamp = self.get_clock().now().to_msg()

        self._marker.ns = '3821_debug'
        self._marker.id = 0
        self._marker.type = Marker.LINE_STRIP
        self._marker.action = Marker.ADD

        self._marker.scale.x = 0.01


    def Begin(self):
        """
        Start drawing a new line, erasing the previous line.
        """
        self._marker.points = []
        self._marker.colors = []
        self._marker.action = Marker.DELETEALL

        self._pub.publish(self._marker)

        self._marker.action = Marker.ADD

    
    def Append(self, x, y):
        """
        Append a point
        """
        a = Point()
        a.x = x
        a.y = y
        
        c = ColorRGBA()
        c.a = 1.0
        c.g = 1.0

        self._marker.points.append(a)
        self._marker.colors.append(c)
    
    def End(self):
        """
        Stop editing and display the result
        """
        self._pub.publish(self._marker)

    

class Map:
    """
    Wrapper class to work easily with occupancy grid
    """
    def __init__(self, map:OccupancyGrid):
        self.width = map.info.width
        self.height = map.info.height
        self.resolution = map.info.resolution
        self.map = map.data
        self.origin = [map.info.origin.position.x, map.info.origin.position.y, map.info.origin.position.z]
    
    def Cost_i(self, x, y):
        """
        Get the cost. Arguments are x, y indices. 
        """
        return self.map[(int)((self.width * y) + x)]
    
    def Cost_f(self, x, y):
        """
        Get the cost. Arguments are x, y world coordinates.
        """
        x_ = (int)((x - self.origin[0]) / self.resolution)
        y_ = (int)((y - self.origin[1]) / self.resolution)

        return self.Cost_i(x_, y_)
    
    def ToIndices(self, x, y) -> PointI:
        """
        Convert world coordinate into x,y indices. Assumes arguments are valid.
        """
        a = PointI()
        a.x = (int)((x - self.origin[0]) / self.resolution)
        a.y = (int)((y - self.origin[1]) / self.resolution)
        return a
    
    def ToCoordinates(self, x, y) -> Point:
        """
        Convert indices into x, y world coordinates. Assumes arguments are valid.
        """
        a = Point()
        a.x = x * self.resolution + self.origin[0]
        a.y = y * self.resolution + self.origin[1]
        return a


class RosNode(Node):
    """
    Do not modify anything other than MakePath
    """

    def __init__(self):
        super().__init__('node_3821')

        self.mapSub = self.create_subscription(OccupancyGrid, '/map', self.OnMapPub, 10)
        self.odomSub = self.create_subscription(Odometry, '/odom', self.OnOdomPub, 10)
        self.pointSub = self.create_subscription(PointStamped, '/clicked_point', self.OnPointPub, 10)

        self.lineDrawer = LineDrawer()

        self.robotPosition = Point()
        self.receiveData = True

    
    def OnMapPub(self, data:OccupancyGrid):
        print('Map Obtained.')

        o = [data.info.origin.position.x, data.info.origin.position.y, data.info.origin.position.z]
        self.map = Map(data)
        print(f'Width: {self.map.width}, Height: {self.map.height}, Resolution: {self.map.resolution}')
        print(f'Origin: {o}')

    def OnOdomPub(self, data:Odometry):
        if not self.receiveData:
            return  
        
        self.robotPosition.x = data.pose.pose.position.x
        self.robotPosition.y = data.pose.pose.position.y
        self.robotPosition.z = data.pose.pose.position.z

    def OnPointPub(self, data:PointStamped):

        if not self.receiveData:
            return    
        receiveData = False

        position = [data.point.x, data.point.y, data.point.z]
        grid = self.map.ToIndices(position[0], position[1])
        grid = [grid.x, grid.y]

        print(f'published point: {position}')
        print(f'Converted: {grid}')
        print(f'Cost: {self.map.Cost_f(position[0], position[1])}')

        self.MakePath(self.robotPosition, data.point)


    def MakePath(self, s:Point, e:Point):
        start = self.map.ToIndices(s.x, s.y)
        end = self.map.ToIndices(e.x, e.y)

        self.lineDrawer.Begin()

        ############################################
        # Implement a path planning algorithm here #




        self.lineDrawer.Append(s.x, s.y)
        self.lineDrawer.Append(e.x, e.y)




        ############################################

        self.lineDrawer.End()
        receiveData = True


def main(args=None):
    rclpy.init(args=args)

    node = RosNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()