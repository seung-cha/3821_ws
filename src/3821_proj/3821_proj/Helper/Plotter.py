import matplotlib.pyplot as plt
import multiprocessing
from multiprocessing import Process



def _ShowMap(map, title):

    plt.title(label=title)
    plt.imshow(map)
    plt.show()

def ShowMap(map, title='untitled'):
    """
    Display the Map class.
    Parse Map.PlotMap() as argument.
    """

    p = Process(target=_ShowMap, args=(map,title))
    p.start()