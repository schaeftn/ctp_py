from os import walk
from os.path import isfile, join
import sys
import itertools

import cv2
import numpy as np
import matplotlib.pyplot as plt
import json
from io import StringIO

sys.path.append('/home/tristan/projects/ctp_py/src/org/combinators/ctp/py/plotting')

currentpath = "/home/tristan/projects/ctp_py/src/org/combinators/ctp/py/plotting"

scaleX = 20
scaleY = 20


def getXCoord(v, dimsize):
    return int((v + dimsize / 2.0) * scaleX)


def getYCoord(v, dimsize):
    return int((dimsize - (v + dimsize / 2.0)) * scaleY)


rmEdgesStyle = dict(color=(0, 0, 0), thickness=7) #tri, grid
#rmEdgesStyle = dict(color=(0, 0, 0), thickness=2) #vcd
rmNodesStyle = dict(color=(223, 60, 60), thickness=-1, radius=20) # Tri
# rmNodesStyle = dict(color=(223, 60, 60), thickness=-1, radius=10) # Grid
#rmNodesStyle = dict(color=(223, 60, 60), thickness=-1, radius=15) # vcd
pathStyle = dict(color=(50, 50, 255), thickness=20)
obstaclesStyle = dict(color=(255, 255, 0), thickness=20)
cellsStyle = dict(color=(116, 184, 116), thickness=7)

for root, dirs, files in walk(currentpath):
    onlyfiles = [root + "/" + f for f in files if isfile(join(root, f)) and f.endswith(".rm.json")]

    print(f"{onlyfiles}")

    for f in [open(f) for f in onlyfiles]:
        data = f.read()
        js = json.loads(data)
        xsize = js['boundaries'][0]
        ysize = js['boundaries'][1]

        nodesImage = 255 * np.ones(shape=[int(scaleX * xsize), int(scaleY * ysize), 3], dtype=np.uint8)
        edgesImage = 255 * np.ones(shape=[int(scaleX * xsize), int(scaleY * ysize), 3], dtype=np.uint8)
        pathImage = 255 * np.ones(shape=[int(scaleX * xsize), int(scaleY * ysize), 3], dtype=np.uint8)
        cellsImage = 255 * np.ones(shape=[int(scaleX * xsize), int(scaleY * ysize), 3], dtype=np.uint8)

        for e in js['roadmap']['edges']:
            cv2.line(edgesImage, pt1=(getXCoord(e[0][0], xsize), getYCoord(e[0][1], ysize)),
                     pt2=(getXCoord(e[1][0], xsize), getYCoord(e[1][1], ysize)),
                     color=rmEdgesStyle['color'], thickness=rmEdgesStyle['thickness'])

        for n in js['roadmap']['nodes']:
            cv2.circle(nodesImage, center=(getXCoord(n[0], xsize), getYCoord(n[1], ysize)),
                       color=rmNodesStyle['color'], radius=rmNodesStyle['radius'], thickness=rmNodesStyle['thickness'])

        for e in zip(js['gpath'], itertools.islice(js['gpath'], 1, None)):
            cv2.line(pathImage, pt1=(getXCoord(e[0][0], xsize), getYCoord(e[0][1], ysize)),
                     pt2=(getXCoord(e[1][0], xsize), getYCoord(e[1][1], ysize)), color=pathStyle['color'],
                     thickness=pathStyle['thickness'])

        for fc in js['freeCells']:
            vertexList = [[getXCoord(js['vertices'][i][0], xsize), getYCoord(js['vertices'][i][1], ysize)] for i in fc]
            secondList = vertexList.copy()
            secondList.append(secondList.pop(0))
            zList = zip(vertexList, secondList)

            print(f"{np.array(vertexList)}")
            hull = cv2.convexHull(np.array(vertexList))
            cv2.drawContours(cellsImage, [hull], 0, color=cellsStyle['color'], thickness=cellsStyle['thickness'])

            # for vPair in zList:
            #     cv2.line(cellsImage, pt1=(getXCoord(vPair[0][0], xsize), getYCoord(vPair[0][1], ysize)),
            #              pt2=(getXCoord(vPair[1][0], xsize), getYCoord(vPair[1][1], ysize)),
            #              color=cellsStyle['color'], thickness=cellsStyle['thickness'])


        plt.imsave(f.name + ".nodes.png", nodesImage)
        plt.imsave(f.name + ".edges.png", edgesImage)
        plt.imsave(f.name + ".path.png", pathImage)
        plt.imsave(f.name + ".cells.png", cellsImage)
