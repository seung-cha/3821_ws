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


from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupTransform
from geometry_msgs.msg import TransformStamped      # Result of lookup transform


import numpy as np  # Array reshaping
from .Helper import Plotter
from .Helper.Heap import MinHeap
from .Helper.PointI import PointI

import math

import sys


# Radius of an arbitrary cylinder that represents the robot, in metres (as rviz uses metres)
ROBOT_RADIUS = 0.15



 
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
    
    
    def PlotMap(self):
        """
        Use this with Plotter.ShowMap to visualiser the occupancy grid.
        """

        # For some reason directly converting into 2D array using np doesn't work.
        # I need to rotate and then flip it.
        l = np.array(self.map).reshape(self.height, -1)
        l = np.rot90(l)
        l = np.fliplr(l)

        return l.tolist()

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
        return (point.x < self.width) and (point.y < self.height) and (point.x >= 0) and (point.y >= 0)
    

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
    You probably do not need to modify anything other than MakePath.
    """

    def __init__(self):
        super().__init__('node_3821')

        # Two variables that might be useful.
        self.robotPosition = Point()    # Current coordinates of the robot
        self.Robot_Cell_Radius = 0      # Robot radius expressed in terms of cells (will update once map is acquired)

        self.mapSub = self.create_subscription(OccupancyGrid, '/map', self.OnMapPub, 10)
        self.odomSub = self.create_subscription(Odometry, '/odom', self.OnOdomPub, 10)
        self.pointSub = self.create_subscription(PointStamped, '/clicked_point', self.OnPointPub, 10)
        self.initPosSub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.OnInitPosePub, 10)
        self.pathPub = self.create_publisher(PoseArray, '/path_points', 10)

        self.lineDrawer = LineDrawer()
        self.receiveData = True


        # TF
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)



    def OnInitPosePub(self, data:PoseWithCovarianceStamped):
        print('Pose obtained.')

        o = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
        r = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]

        print('Initial Pose published: ')
        print(f'Position: {o}')
        print(f'Orient: {r}')
    
    def OnMapPub(self, data:OccupancyGrid):
        print('Map Obtained.')

        self.map = Map(data)
        self.Robot_Cell_Radius = self.map.ToIndices(x= ROBOT_RADIUS + self.map.origin[0], y= self.map.origin[1]).x

        o = [data.info.origin.position.x, data.info.origin.position.y, data.info.origin.position.z]
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
        #  Algorithm must return a list of PointI  #
        #        and It should be named path       #
        ############################################

        pf = APF(start, end, self.map)

        # Show various maps
        # Plotter.ShowMap(self.map.PlotMap(), 'Map')
        # pm = pf.PotentialMap()
        # Plotter.ShowCostMap(pm.PlotMap(), 'Potential Map')
        # rm = pf.RepulsiveMap()
        # Plotter.ShowCostMap(rm.PlotMap(), 'Repulsive Map')
        # tm = pf.TotalPotentialMap()
        # Plotter.ShowCostMap(tm.PlotMap(), 'Total Potential')
        # (x,y) = pf.PotentialGradientMap()
        # Plotter.ShowGradientArrow(y.PlotMap(), x.PlotMap())
        # (x1,y1) = pf.RepulsiveGradientMap()
        # Plotter.ShowGradientArrow(y1.PlotMap(), x1.PlotMap(), "Replusive Gradient")
        # (x2,y2) = pf.TotalGradientMap()
        # Plotter.ShowGradientArrow(y2.PlotMap(), x2.PlotMap(), "Total Gradient")


        path:list[PointI]
        path = pf.Run()

        Plotter.ShowPathMap(self.map.PlotMap(), path)
        ############################################

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


        self.lineDrawer.End()
        receiveData = True

    

class APF:
    """
    Implementation of artificial Potential Field
    """

    def __init__(self, start:PointI, end:PointI, map:Map):
        self.start = start
        self.end = end
        self.map = map
        self.obstacleDistMap = self.BushFireDistance()

        # Define parameters
        self.ITERATION_LIMIT = 100000     # Maximum number of iteration allowed
        self.DISTANCE_TOLERANCE = 0.15   # Threshold at which the robot is considered at the destination


        self.ATTRACTIVE_CONST = 0.01 # Constant for attractive potential
        self.REPULSIVE_CONST = 0.0005 # Constant for repulsive potential

        # Both assumes to be expressed in metre
        self.ATTRACTIVE_DIST = 0.5 # Range at which the robot is considered close to the goal
        self.REPULSIVE_DIST = ROBOT_RADIUS * 1.5 # Range at which the robot is considered close to an obstacle.
        # Make sure the replusive force acts on the robot early so that it has room to rotate away

        # corresponding number of cells
        self.REPULSIVE_DIST_CELL = self.map.ToIndices(self.map.origin[0] + self.REPULSIVE_DIST, self.map.origin[1]).x



    def Run(self):
        """
        Gradient Descendent Method
        """
        p = self.start  # Current point expressed in cell coordinate
        path = [p]
        iter = 0

        while iter < self.ITERATION_LIMIT and self.Dist(p, self.end) > self.DISTANCE_TOLERANCE:

            # new point = old point - gradient at old point
            pGradient = self.TotalGradient(p)
            pWorld = self.map.ToCoordinates(p.x, p.y)
            p_new = Point()

            p_new.x = pWorld.x - pGradient.x
            p_new.y = pWorld.y - pGradient.y

            # Convert the world coordinate back to cell coordinate
            p_cell = self.map.ToIndices(p_new.x, p_new.y)

            if not self.map.Valid(p_cell):
                print(f'not valid: {p_cell.x}, {p_cell.y}')
            p = p_cell
            path.append(p_cell)
            iter = iter + 1



        print(f'Number of Iteration: {iter}')
        if iter == self.ITERATION_LIMIT:
            print('ITERATION LIMIT REACHED')


        return path
        




    def PotentialMap(self):
        pMap = self.map.CopySet(0.0)

        for x in range(pMap.width):
            for y in range(pMap.height):
                currentPoint = PointI(x,y)
                pMap.SetCost(x, y, self.AttractivePotential(currentPoint))
        
        return pMap
    

    def PotentialGradientMap(self):
        xMap = self.map.CopySet(0.0)
        yMap = self.map.CopySet(0.0)

        for x in range(self.map.width):
            for y in range(self.map.height):
                currentPoint = PointI(x,y)
                g = self.AttractiveGradient(currentPoint)
                xMap.SetCost(x, y, g.x)
                yMap.SetCost(x, y, g.y)
        
        return (xMap, yMap)
    

    def RepulsiveMap(self):
        pMap = self.map.CopySet(0.0)

        for x in range(pMap.width):
            for y in range(pMap.height):
                currentPoint = PointI(x,y )
                pMap.SetCost(x, y, self.RepulsivePotential(currentPoint))

        return pMap


    def RepulsiveGradientMap(self):
        xMap = self.map.CopySet(0.0)
        yMap = self.map.CopySet(0.0)

        for x in range(self.map.width):
            for y in range(self.map.height):
                currentPoint = PointI(x,y)
                g = self.RepulsiveGradient(currentPoint)
                xMap.SetCost(x, y, g.x)
                yMap.SetCost(x, y, g.y)

        return (xMap, yMap)
    

    def TotalPotentialMap(self):
        potential = self.PotentialMap()
        repulsive = self.RepulsiveMap()

        for x in range(potential.width):
            for y in range(potential.height):
                potential.SetCost(x, y, potential.Cost_i(x, y) + repulsive.Cost_i(x, y))
        
        return potential
    
    def TotalGradientMap(self):
        (px, py) = self.PotentialGradientMap()
        (rx, ry) = self.RepulsiveGradientMap()

        for x in range(px.width):
            for y in range(px.height):
                px.SetCost(x, y, (px.Cost_i(x, y) + rx.Cost_i(x, y)))
                py.SetCost(x, y, (py.Cost_i(x, y) + ry.Cost_i(x, y)))
        return (px, py)

    def BushFireDistance(self):
        """
        Compute the distance from each cell to nearest obstacle using bush fire algorithm.
        """

        # Occupancy grid is interanally 8 bit unsigend int.
        # Int overflows quite quickly with this data type so make a hard copy with more bits.
        bushMap = self.map.CopySet(0)
        visitedMap = self.map.CopySet(False)
        queue = MinHeap()   # Use heap to fire spread equally. Double buffer approach is probably more efficient.

        for x in range(self.map.width):
            for y in range(self.map.height):
                p = PointI(x, y)

                if self.map.Cost_i(x, y) >= 1:
                    bushMap.SetCost(x, y, 1)    # Occupancy Grid has range of [0, 100]
                    visitedMap.SetCost(x, y, True)

                    # 8 connectivity
                    neighbours = [
                    PointI(p.x + 1, p.y), PointI(p.x - 1, p.y), PointI(p.x, p.y + 1), PointI(p.x, p.y - 1),
                    PointI(p.x + 1, p.y + 1), PointI(p.x - 1, p.y + 1), PointI(p.x - 1, p.y - 1), PointI(p.x + 1, p.y - 1)
                            ]
                    
                    # Consider all neighbour cells of occupied grid and push it to queue if it's 0.
                    for neighbour in neighbours:
                        if self.map.Valid(PointI(neighbour.x, neighbour.y)) and self.map.Cost_i(neighbour.x, neighbour.y) == 0:
                            queue.Push(0, neighbour.x, neighbour.y)


        # Run bush fire.
        iteration = 1
        while len(queue) != 0:
            [_, p] = queue.Pop()

            p = PointI(p[0], p[1])  # Convert tuple to PointI

            # Skip if cell is already visitied
            if visitedMap.Cost_i(p.x, p.y):
                continue

            visitedMap.SetCost(p.x, p.y, True)

            minCost = sys.maxsize
            # 8 connectivity
            neighbours = [
            PointI(p.x + 1, p.y), PointI(p.x - 1, p.y), PointI(p.x, p.y + 1), PointI(p.x, p.y - 1),
            PointI(p.x + 1, p.y + 1), PointI(p.x - 1, p.y + 1), PointI(p.x - 1, p.y - 1), PointI(p.x + 1, p.y - 1)
                    ]
            
            for neighbour in neighbours:
                if not bushMap.Valid(PointI(neighbour.x, neighbour.y)):
                    continue

                if  bushMap.Cost_i(neighbour.x, neighbour.y) == 0:
                    # Push non visited neighbour to queue
                    queue.Push(iteration, neighbour.x, neighbour.y)
                else:
                    bushCost = bushMap.Cost_i(neighbour.x, neighbour.y)
                    if minCost > bushCost:
                        minCost = bushCost
            
            bushMap.SetCost(p.x, p.y, minCost + 1)
            iteration = iteration + 1

        return bushMap

            
    def Dist(self, p1:PointI, p2:PointI):
        """
        Distance between two points represented in world coordinate
        """
        p1_ = self.map.ToCoordinates(p1.x, p1.y)
        p2_ = self.map.ToCoordinates(p2.x, p2.y)

        return math.dist([p1_.x, p1_.y], [p2_.x, p2_.y])


    def TotalPotential(self, point:PointI):
        p = self.AttractivePotential(point)
        r = self.RepulsivePotential(point) 
        return p + r
    
    def TotalGradient(self, point:PointI):
        p = self.AttractiveGradient(point)
        r = self.RepulsiveGradient(point)

        po = Point()
        po.x = p.x + r.x
        po.y = p.y + r.y

        return po


    def RepulsivePotential(self, point:PointI):
        """
        Repulsive potential at given cell which is provided as follows:

        P(x) = 1/2*c*(1/d - 1/Q)^2
        where c is repulsive constant, d is the distance from the cell to obstacle and Q is the effect radius
        """
        # Potential is 0 if there are no obstacles within range of effect.
        if self.obstacleDistMap.Cost_i(point.x, point.y) > self.REPULSIVE_DIST_CELL or self.map.Cost_i(point.x, point.y) >= 1:
            return 0.0
        else:
            # Get the list of obstacles cells that are within the range
            cells = self.GetObstacleCells(point, self.REPULSIVE_DIST_CELL)
            
            val = 0.0
            for cell in cells:
                dist = self.Dist(point, cell)
                val = val + ( (self.REPULSIVE_CONST / 2) * (1/dist - 1/self.REPULSIVE_DIST)**2)

        
        return val
    
    def RepulsiveGradient(self, point:PointI):
        """
        Repulsive gradient at a given cell which is as follows:
        c * (1/Q - 1/d) * 1/d^2 * D

        where D = norm(point - obstacle)
        """
        if self.obstacleDistMap.Cost_i(point.x, point.y) > self.REPULSIVE_DIST_CELL or self.map.Cost_i(point.x, point.y) >= 1:
            # return 0 gradient
            p = Point()
            p.x = 0.0
            p.y = 0.0
            return p
        else:
            cells = self.GetObstacleCells(point, self.REPULSIVE_DIST_CELL) 

            p = Point()
            p.x = 0.0
            p.y = 0.0

            # Sum gradient produced by each cell
            for cell in cells:
                dist = self.Dist(point, cell)

                const = self.REPULSIVE_CONST * (1/self.REPULSIVE_DIST - 1/dist) * (1/(dist**2))

                p0 = self.map.ToCoordinates(point.x, point.y)
                p1 = self.map.ToCoordinates(cell.x, cell.y)

                dx = const * (p0.x - p1.x) / dist
                dy = const * (p0.y - p1.y) / dist

                p.x = p.x + dx
                p.y = p.y + dy

        return p

            

            
            










    def GetObstacleCells(self, point:PointI, radius):
        """
        Search for all obstacle cells around the point within the radius.
        Assume the point itself is not an obstacle.
        """
        cells = []

        for x in range(point.x - radius, point.x + radius + 1):
            for y in range(point.y - radius, point.y + radius + 1):
                if self.map.Valid(PointI(x, y)) and self.map.Cost_i(x, y) >= 1:
                    cells.append(PointI(x, y))
        
        return cells


    def AttractivePotential(self, point:PointI):
        """
        Attractive potential at given cell
        """

        d = self.Dist(point, self.end)

        if d <= self.ATTRACTIVE_DIST:
            # Use quadratic
            return self.QuadraticPotential(point)
        else:
            # attractive_dist * conical - quadratic
            c = self.ConicalPotential(point)
            return self.ATTRACTIVE_DIST * c - self.QuadraticPotential(point)



    def AttractiveGradient(self, point:PointI):
        """
        Gradient of attractive potential
        """

        d = self.Dist(point, self.end)

        if d <= self.ATTRACTIVE_DIST:
            # quadratic gradient
            return self.QuadraticGradient(point)
        else:
            # attractive_dist * conical gradient
            p = self.ConicalGradient(point)
            p.x = p.x * self.ATTRACTIVE_DIST
            p.y = p.y * self.ATTRACTIVE_DIST

            return p



    def ConicalPotential(self, point:PointI):
        """
        Conical potential of given point.
        Given a point p, it is defined as follows:

        U(p) = c * d

        where c is attractive potential constant and d is the distance from p to the goal
        """
        # Might convert from cell coordinate to world coordinate later.
        return self.ATTRACTIVE_CONST * self.Dist(point, self.end)

    def ConicalGradient(self, point:PointI) -> Point:
        """
        Gradient of conical potential of given point (for euclidean distance).
        For a point p, it is defined as follows:

        DU(p) = c/d * (p - end)
        """
        const = self.ATTRACTIVE_CONST/ self.Dist(point, self.end)
        pointDiff = PointI(x=point.x - self.end.x, y=point.y - self.end.y)

        p = Point()
        p.x = pointDiff.x * const
        p.y = pointDiff.y * const
        return p

    def QuadraticPotential(self, point:PointI):
        """
        Quadratic potential of a given point.
        Defined as follows:

        U(p) = c/2 d^2
        """
        return self.ATTRACTIVE_CONST/2 * self.Dist(point, self.end)**2


    def QuadraticGradient(self, point:PointI) -> Point:
        """
        Gradient of quadratic potential. Defined as

        DU(p) = c*(p - end)
        """
        pointDiff = PointI(x=point.x - self.end.x, y=point.y - self.end.y)
        p = Point()
        p.x = pointDiff.x * self.ATTRACTIVE_CONST
        p.y = pointDiff.y * self.ATTRACTIVE_CONST
        return p

class A_star:
    """
    Implementation of A* algorithm.
    """
    
    def __init__(self, start:PointI, end:PointI, map:Map):
        self.start = start
        self.end = end
        self.map = map

    def Heuristic(self, point:PointI):
        """
        Manhattan distance, sum of absolute value difference in each axis.
        """
        return abs(point.x - self.end.x) + abs(point.y - self.end.y)



    def Run(self):
        # create a map to store the cost to visit each cell.
        costMap = self.map.CopySet(float('inf'))            # cost map of the same size
        costMap.SetCost(self.start.x, self.start.y, 0.0)    # Starting cell is reachable without cost.

        # create a map to store the predecessor of each cell.
        # Each cell will take store PointI that represents the cell coordinates of the predecessor.
        predMap = self.map.CopySet(None)

        # Store the explored cells, ordered by increasing cost.
        frontier = MinHeap()                                # Min heap frontier
        frontier.Push(0.0, self.start.x, self.start.y)
 

        while len(frontier) > 0:
            # Get the cost and coordinates of the root in the min heap
            (c_Cost, c_Vertex) = frontier.Pop()
            
            # convert tuple into PointI to make it easier to work with
            c_Vertex = PointI(c_Vertex[0], c_Vertex[1])

            if c_Vertex == self.end:
                break

            # First line is the 4 vertical and horizontal neighbours.
            # Second line is the 4 diagonal neighbours.
            neighbours =[
                PointI(c_Vertex.x + 1, c_Vertex.y), PointI(c_Vertex.x - 1, c_Vertex.y), PointI(c_Vertex.x, c_Vertex.y + 1), PointI(c_Vertex.x, c_Vertex.y - 1),
                PointI(c_Vertex.x + 1, c_Vertex.y + 1), PointI(c_Vertex.x - 1, c_Vertex.y + 1), PointI(c_Vertex.x + 1, c_Vertex.y - 1), PointI(c_Vertex.x - 1, c_Vertex.y - 1)
                         ]


            for i in range(len(neighbours)):
                # Don't do anything if the coordinates are invalid or if there is an obstacle.
                if not self.map.Valid(neighbours[i]) or self.map.Cost_i(neighbours[i].x, neighbours[i].y) >= 1.0:
                    continue
                
                cost = c_Cost + self.Heuristic(neighbours[i])

                # Update the cell if the distance is shorter than the stored one.
                if cost < costMap.Cost_i(neighbours[i].x, neighbours[i].y):
                    costMap.SetCost(neighbours[i].x, neighbours[i].y, cost)
                    frontier.Push(cost, neighbours[i].x, neighbours[i].y)

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


        # Show the supplied map
        Plotter.ShowMap(self.map.PlotMap(), 'Map')

        # Display the cost map in pyplot.
        Plotter.ShowCostMap(costMap.PlotMap(), 'Cost')

        # Display the generated path on the world map
        Plotter.ShowPathMap(self.map.PlotMap(), path)
        

        return path

                    


                















def main(args=None):
    rclpy.init(args=args)

    node = RosNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()