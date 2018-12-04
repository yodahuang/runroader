import numpy as np
import matplotlib.pyplot as plt
import math
import motion_model
import ExportJson
import copy
import os
from collections import defaultdict
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import queue
import functools
from shapely.geometry import Point as SPoint
from shapely.geometry.polygon import Polygon as SPolygon


# Each obstacle is s1, s2, t1, t2, = [[30. , 5],[40,  5],[40. , 10],[40. , 5]] np array
# obstacle list  is list of these arrays
class Environment:
    def __init__(self, obstacleList):
        self.obstacleList = obstacleList
        self.polyList = []
        for i in range(len(self.obstacleList)):
            poly = [tuple(x) for x in list(self.obstacleList[i])]
            self.polyList.append(SPolygon(poly))

    def CheckIfInsidePolygon( self, config, polygon):
        spoint = SPoint(point[0], point[1])
        return polygon.contains(spoint)

    # Each point is a list of s,t, returns the top left s,t of the obstacle we collide with
    def CheckCollision(self, point1, point2):

        point1X = point1[0]
        point1Y = point1[1]

        point2X = point2[0]
        point2Y = point2[1]

        if np.abs(point1X - point2X) > np.abs(point1Y - point2Y):
            nSteps = int(np.floor(np.abs(point1X - point2X) / self.collisionStep))
            x = np.linspace(point1X, point2X, nSteps)
            xIn = [point1X, point2X]
            yIn = [point1Y, point2Y]

            if np.all(np.diff(x) > 0):
                y = np.interp(x, xIn, yIn)
            else:
                yIn = yIn[::-1]
                xIn = xIn[::-1]
                x = x[::-1]

                y = np.interp(x, xIn, yIn)

        else:
            nSteps = int(np.floor(np.abs(point1Y - point2Y) / self.collisionStep))
            y = np.linspace(point1Y, point2Y, nSteps)
            xIn = [point1X, point2X]
            yIn = [point1Y, point2Y]

            if np.all(np.diff(y) > 0):
                x = np.interp(y, yIn, xIn)
            else:
                yIn = yIn[::-1]
                xIn = xIn[::-1]
                y = y[::-1]

                x = np.interp(y, yIn, xIn)


        for j in range(len(self.obstacleList)):
            for i in range(y.shape[0]):
                if self.CheckIfInsidePolygon([x[i], y[i]], polygon=self.sPoly[j]):
                    return np.array([self.obstacleList[j][0], self.obstacleList[j][3]])

        return False


