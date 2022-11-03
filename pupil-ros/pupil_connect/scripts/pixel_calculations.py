from screeninfo import get_monitors
import math
import numpy
import pygame

if __name__ == '__main__':
    monitor = get_monitors()
    pixel_width = monitor[0].width_mm/monitor[0].width
    pixel_height = monitor[0].height_mm/monitor[0].height
    pixel_diag = math.sqrt(pow(monitor[0].width_mm, 2) + pow(monitor[0].height_mm, 2))/math.sqrt(pow(monitor[0].width, 2) + pow(monitor[0].height, 2))
    # print('width: ', pixel_width)
    # print('height: ', pixel_height)
    # print('diag: ', pixel_diag)
    test = numpy.zeros((5, 3))
    origin = (1, 2)
    test[:, :2] = origin
    #print(test)
    bla = numpy.linalg.norm([0, 2], 2)
    #print(bla)
    baba = numpy.array([0, 0, 2])
    testlist = []
    testlist.append((0, 0))
    test2 = numpy.asarray([testlist[0], 0])
    # print(numpy.arccos(0.3))
    # print(numpy.square(2))
