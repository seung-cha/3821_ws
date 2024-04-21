import matplotlib.pyplot as plt
import matplotlib.colors

import multiprocessing
from multiprocessing import Process

from .PointI import PointI

import numpy as np



def _ShowMap(map, title):

    plt.title(label=title)
    plt.imshow(map, cmap='binary')
    plt.show()

def ShowMap(map, title='Map'):
    """
    Display the Map class.
    Parse Map.PlotMap() as argument.
    """

    p = Process(target=_ShowMap, args=(map,title))
    p.start()


def _ShowCostMap(map, title):

    plt.title(label=title)
    plt.imshow(map)
    plt.colorbar()
    plt.show()

def ShowCostMap(costMap, title='Cost Map'):
    """
    Display the cost map.
    Parse Map.PlotMap() as argument.
    """

    p = Process(target=_ShowCostMap, args=(costMap,title))
    p.start()


def _ShowPathMap(map, title):

    cmap = matplotlib.colors.ListedColormap(['white', 'black'])
    cmap.set_bad('green')

    plt.title(label=title)
    plt.imshow(map, cmap=cmap)
    plt.show()


def ShowPathMap(map, path, title='Generated Path'):
    """
    Display the path on the map.
    Parse Map.PlotMap() for map.
    path must be a list of PointI
    """
    
    l = np.array(map).astype(float)
    # Undo the array transformation
    l = np.fliplr(l)
    l = np.rot90(l, 3)

    # Mark the path
    point: PointI
    for point in path:
        l[point.y][point.x] = np.nan

    print("brh")
    # Do the trnasform again
    l = np.rot90(l)
    l = np.fliplr(l)

    p = Process(target=_ShowPathMap, args=(l,title))
    p.start()



def _ShowGradientArrow(xMap, yMap, title):
    plt.title(title)
    plt.quiver(xMap, yMap)
    plt.show()

def ShowGradientArrow(xMap, yMap, title='Gradient Map'):
    """
    Display the gradient arrow.
    xMap and yMap must be n dimensional array with equal size
    """

    p = Process(target=_ShowGradientArrow, args=(xMap, yMap, title))
    p.start()