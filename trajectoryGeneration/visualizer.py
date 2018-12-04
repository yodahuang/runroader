import itertools
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
import copy
import csv
import os
import time
import pickle
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection



class Visualizer():
    def __init__(self, xRange=[-5.,5.], yRange=[-5.,5.]):
        self.xRange = xRange
        self.yRange = yRange
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)

    def PlotPoints(self, points):
        x = points[:, 0]
        y = points[:, 1]

        plt.scatter(x, y)

    def PlotPoint(self, point, color = '-g'):
        x = point[0]
        y = point[1]

        plt.scatter(x, y)

    def PlotPath(self, path):
        path = np.array(path)

        x = path[:, 0]
        y = path[:, 1]

        plt.plot(x, y, "-r")

    def plotPathPointArray(self, x, y):
        plt.plot(x, y, "-r")

    def plotVisibility(self, point, neighbors):

        for i in range(neighbors.shape[0]):
            x = np.array([point[0], neighbors[i, 0]])
            y = np.array([point[1], neighbors[i, 1]])
            plt.plot(x, y, "-r")


    def ShowPlot(self):
        plt.axis("equal")
        plt.xlim(self.xRange[0], self.xRange[1])
        plt.ylim(self.yRange[0], self.yRange[1])
        plt.show()

    # polygons is a list of N,2 arrays containg x and y coords of polygon
    def PlotPolygons(self, polygons, alpha = 0.1, color=[0,1,0]):
        patches=[]
        for polygon in polygons:
            patches.append(Polygon(polygon, True, fill=True))

        # colors = 1 * color * np.ones(len(patches))
        p = PatchCollection(patches, alpha=alpha, match_original=True)
        p.set_color(color)
        # p.set_array(np.array(colors))
        self.ax.add_collection(p)


def main():

    visualizer = Visualizer()

    N=3
    E=3
    polygons=[]
    for i in range(N):
        polygon = 5*np.random.rand(E,2)
        polygons.append(polygon)

    visualizer.PlotPolygons(polygons,False)
    visualizer.ShowPlot()

if __name__ == '__main__':
    main()