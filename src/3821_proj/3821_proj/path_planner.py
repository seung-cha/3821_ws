#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Header

from nav_msgs.msg import Odometry                   # Required by /odom
from nav_msgs.msg import OccupancyGrid              # Required by /map
from geometry_msgs.msg import PointStamped          # Required by /clicked_point
from geometry_msgs.msg import PoseWithCovarianceStamped # Required by /initialpose

from geometry_msgs.msg import PoseArray             # Publish a set of points
from geometry_msgs.msg import Pose

from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray      # Message to publish debugging message
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from geometry_msgs.msg import Point                 # Alternative data structure

import copy                                         # Deep copy

import math
import heapq                                        # Priority Queue

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupTransform
from geometry_msgs.msg import TransformStamped      # Result of lookup transform



class PointI:
    """
    Accepts integers
    """

    def __init__(self, x = 0, y = 0, z = 0):
        self.x = x
        self.y = y
        self.z = z

    def __eq__(self, point):
        return (self.x == point.x) and (self.y == point.y) and (self.z == point.z)
    
    def __lt__(self, point):
        return True
    


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
    
    def SetCost(self, x, y, value):
        """
        Set the cost of the cell at [x, y]. Assumes the arguments are valid.
        """
        self.map[(int)((self.width * y) + x)] = value
    
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
    
    def CopyBlank(self):
        """
        Create a deep copy of this object. Each cell of the grid = 0.
        """
        o = copy.deepcopy(self)
        
        for i in range(len(o.map)):
            o.map[i] = 0
        
        return o

    def Copy(self):
        """
        Create a deep copy of this object.
        """
        return copy.deepcopy(self)
    
    def Valid(self, point: PointI):
        """
        Check if argument is within the size of the map
        """
        return (point.x < self.width) and (point.y < self.height)
    

    def CopySet(self, value):
        """
        Create a deep copy of this object and assign each with the the given value
        """

        o = copy.deepcopy(self)
        o.map = []
        for i in range(len(self.map)):
            o.map.append(value)
        
        return o


class RosNode(Node):
    """
    Do not modify anything other than MakePath
    """

    def __init__(self):
        super().__init__('node_3821')

        self.mapSub = self.create_subscription(OccupancyGrid, '/map', self.OnMapPub, 10)
        self.odomSub = self.create_subscription(Odometry, '/odom', self.OnOdomPub, 10)
        self.pointSub = self.create_subscription(PointStamped, '/clicked_point', self.OnPointPub, 10)

        # We will use this instead when deploying to a physical robot
        self.initPosSub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.OnInitPosePub, 10)

        self.pathPub = self.create_publisher(PoseArray, '/path_points', 10)

        self.lineDrawer = LineDrawer()

        self.robotPosition = Point()

        self.receiveData = True


        # TF
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)




    def OnInitPosePub(self, data:PoseWithCovarianceStamped):
        
        #To do: Use this pose instead when we are implementing for the physical robot
        print('Pose obtained.')

        o = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
        r = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]

        # self.robotPosition.x = data.pose.pose.position.x
        # self.robotPosition.y = data.pose.pose.position.y
        # self.robotPosition.z = data.pose.pose.position.z

        print('Initial Pose published: ')
        print(f'Position: {o}')
        print(f'Orient: {r}')
    
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


        tf:TransformStamped
        tf = self.tfBuffer.lookup_transform('map', 'odom', data.header.stamp, Duration(seconds=1))

        robot_P = Point()
        robot_P.x = self.robotPosition.x + tf.transform.translation.x
        robot_P.y = self.robotPosition.y + tf.transform.translation.y
        robot_P.z = self.robotPosition.z + tf.transform.translation.z

        self.MakePath(robot_P, data.point)


    def MakePath(self, s:Point, e:Point):
        start = self.map.ToIndices(s.x, s.y)
        end = self.map.ToIndices(e.x, e.y)

        self.lineDrawer.Begin()

        ############################################
        # Implement a path planning algorithm here #

        a_star = A_star(start, end, self.map)

        path:list[PointI]
        path = a_star.Run()

        print(f'Size of the path: {len(path)}')

        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(path)):
            # Convert from index coordinate to world coordinate
            coords = self.map.ToCoordinates(path[i].x, path[i].y)
            self.lineDrawer.Append(coords.x, coords.y)

            p = Pose()
            p.position.x = coords.x
            p.position.y = coords.y
            msg.poses.append(p)

        self.pathPub.publish(msg)

        # self.lineDrawer.Append(s.x, s.y)
        # self.lineDrawer.Append(e.x, e.y)

        ############################################

        self.lineDrawer.End()
        receiveData = True


    


class A_star:
    
    def __init__(self, start:PointI, end:PointI, map:Map):
        self.start = start
        self.end = end
        self.map = map

    def Heuristic(self, point:PointI):
        """
        Euclidean distance from given point to the destination.
        Could change to Manhattan distance instead later for experiement.
        """
        return math.dist((point.x, point.y), (self.end.x, self.end.y))

    def Run(self):
        costMap = self.map.CopySet(float('inf'))    # cost map of the same size 
        print(f'size of costmap: {len(costMap.map)}')

        costMap.SetCost(self.start.x, self.start.y, 0.0)

        predMap = self.map.CopySet(None)            # predecessor map of the same size

        frontier = []                               # Minheap
        heapq.heappush(frontier, (0.0, self.start)) 

        while len(frontier) > 0:
            current = heapq.heappop(frontier)

            c_Cost:float
            c_Vertex:PointI

            c_Cost = current[0]
            c_Vertex = current[1]

            if c_Vertex == self.end:
                break

            neighbours = [PointI(c_Vertex.x + 1, c_Vertex.y), PointI(c_Vertex.x - 1, c_Vertex.y), PointI(c_Vertex.x, c_Vertex.y + 1), PointI(c_Vertex.x, c_Vertex.y - 1)]

            for i in range(len(neighbours)):
                # Don't do anything if the coordinates are invalid.
                if not self.map.Valid(neighbours[i]) or self.map.Cost_i(neighbours[i].x, neighbours[i].y) >= 1.0:
                    continue

                cost = c_Cost + self.Heuristic(neighbours[i])
                # Only consider if the distance is shorter
                if cost < costMap.Cost_i(neighbours[i].x, neighbours[i].y):
                    costMap.SetCost(neighbours[i].x, neighbours[i].y, cost) # Update the neighbour's cost
                    heapq.heappush(frontier, (cost, neighbours[i]))

                    #Update the predecessor array
                    predMap.SetCost(neighbours[i].x, neighbours[i].y, c_Vertex)

        
        # Establish a path
        path = []
        backTrack = self.end

        while predMap.Cost_i(backTrack.x, backTrack.y) is not None:
            path.append(backTrack)
            backTrack = predMap.Cost_i(backTrack.x, backTrack.y)
        
        # lastly, append the starting vertex and reverse.
        path.append(self.start)
        path.reverse()

        return path

                    


                















def main(args=None):
    rclpy.init(args=args)

    node = RosNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()