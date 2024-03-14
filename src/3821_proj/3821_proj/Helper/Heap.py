import heapq


class _HeapData: 
    """
    Data used in heap, which is a tuple of distance and coordinates.
    """

    def __init__(self, dist, x, y):
        self.dist = dist
        self.x = x
        self.y = y

    def __lt__(self, data):
        return self.dist < data.dist

class MinHeap:
    """
    Wrapper around heapq. 
    """

    def __init__(self):
        self._heap = []

    def __len__(self):
        return len(self._heap)
    
    def Push(self, cost, x, y):
        data = _HeapData(cost, x, y)
        heapq.heappush(self._heap, data)

    def Pop(self):
        """
        Pop the min element as a tuple of (cost, (x, y))
        """
        data = heapq.heappop(self._heap)
        return (data.dist, (data.x, data.y))