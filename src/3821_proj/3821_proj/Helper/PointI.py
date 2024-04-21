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

    def __repr__(self):
        return "(" + str(self.x) + ", " + str(self.y) +")"