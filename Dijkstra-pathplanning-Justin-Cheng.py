#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENPM 661 - Dr. Reza Monfaredi
Project 2
Due date: March 5, 11:59 p.m.

@author: justincheng
"""

import cv2 as cv
import numpy as np
from collections import deque

img = 255*np.ones((251, 401, 3), np.uint8)
nodeCount = 0

interList = []
openList, openListPoints = deque(), deque()
openStage, openStagePoints = deque(), deque()
closedList, closedListPoints = deque(), deque()
solutions = deque()
visited = deque()

finish = False

#####
### Functions
#####

## Input

def getCoordinate(start):
    
    # Print out start or goal commands
    if start:
        string = "starting"
    else:
        string = "goal"
    
    while True:
        
        # Check if within width bounds
        xBound = range(5,396)
        x = int(input("Enter "+string+" X coordinate: \n(Must be between 5 and 395): \n"))
        while True:
            if x in xBound:
                break
            else:
                x = int(input("Error. Please enter an X between 5 and 395.\nEnter "+string+" X coordinate:"))
        
        # Check if within height bounds
        yBound = range(5,246)
        y = int(input("Enter "+string+" Y coordinate: \n(Must be between 5 and 245): \n"))
        while True:
            if y in yBound:
                break
            else:
                y = int(input("Error. Please choose a Y between 5 and 245.\nEnter "+string+" Y coordinate:"))
        
        # Check if the point is over an object
        if not collisionDetection([x,y]):
            break
        else:
            print("Point is on an object. Please enter corrdinates again.\n\n")
        
    return [x, y]

## Collision

def collisionDetection(p):
    
    x, y = p[0], p[1]
    
    return (collidePolygon(x, y) or collideHexagon(x, y) or collideCircle(x, y) or collideBoundary(x, y))
    
def collidePolygon(x, y):
    
    # Lines of Polygon
    L1 = (-0.316 * x) + 71.148 - y
    L2 = (-0.714 * x) + 128.29 - y
    L3 = (3.263 * x) - 213.15 - y
    L4 = (1.182 * x) + 30.176 - y
    L5 = (0.0766 * x) + 60.405 - y
    
    # Check for collision (True or False)
    return ((L1 <= 0 and L2 >= 0 and L5 >= 0) or (L3 <= 0 and L4 >= 0 and L5 <= 0))

def collideHexagon(x, y):
    
    # Lines of Hexagon
    L1 = (-0.577 * x) + 219.28 - y
    L2 = (0.577 * x) - 11.658 - y
    L3 = 240 - x
    L4 = (-0.577 * x) + 311.66 - y
    L5 = (0.577 * x) + 80.718 - y
    L6 = 160 - x
    
    # Check for collision (True or False)
    return (L1 <= 0 and L2 <= 0 and L3 >= 0 and L4 >= 0 and L5 >= 0 and L6 <= 0)

def collideCircle(x, y):
    
    # Circumfrence of Circle
    C = (x - 300)**2 + (y - 65)**2 - (45**2)
    
    # Check for collision (True of False)
    return (C < 0) 

def collideBoundary(x, y):
    
    # Bounrdaries of the frame
    return (x <= 4 or x>= 398 or y <= 5 or y >= 245)

## Algorithm

def Djikstra(node):
    global openStage, openListPoints, closedListPoints
    
    point = [node[0][0], node[0][1]]
    
    # Check if point is not colliding with object or already visited
    if not collisionDetection(node[0]):
        if point not in closedListPoints :
            
            # Check if point is already in list of nodes to be visted
            if point in openStagePoints :
                idx = openStage[openStagePoints.index(point)]
                
                # Check if cost is lower and update cost if so
                if node[3] < idx[2]:
                    idx[2] = node[3]
                    idx[0] = node[1]
            else:
                
                # Add to nodes to be visted next round
                openStage.append([node[1], node[2], node[3]])
                openStagePoints.append([node[0][0], node[0][1]])

## Plotting

def displayPlot():
    
    global img
    
    img = 255*np.ones((251, 401, 3), np.uint8)
    
    # Create Polygon
    poly_points = np.array([[27, 63], [144, 26], [86, 67], [117, 168]],dtype=np.int32)
    cv.fillConvexPoly(img,poly_points, 0)
    
    # Create Hexagon
    hex_points = np.array([[160,127], [200,103], [240,127], [240,173], [200,196], [160,173]],dtype=np.int32)
    cv.fillConvexPoly(img,hex_points, 0)
    
    # Create Circle
    cv.circle(img, (300, 65), 45, (0, 0, 0),-1)

    return img

## Move Functions

def move(node, nodePosition):
    global nodeCount
    
    # Get node information to pass to all potential nodes
    x, y = nodePosition[0], nodePosition[1]
    parentIndex = node[0]
    currentCost = node[2]

    # All node movements
    moveU(x, y, parentIndex, currentCost)
    moveUR(x, y, parentIndex, currentCost)
    moveR(x, y, parentIndex, currentCost)
    moveDR(x, y, parentIndex, currentCost)
    moveD(x, y, parentIndex, currentCost)
    moveDL(x, y, parentIndex, currentCost)
    moveL(x, y, parentIndex, currentCost)
    moveUL(x, y, parentIndex, currentCost)
    
def moveR(x, y, parentIndex, currentCost):
    global nodeCount
    
    nodeCount += 1
    index = nodeCount
    cost = currentCost + 1.0
    
    newNode = [(x+1,y), index, parentIndex, cost]
    Djikstra(newNode)
    
def moveL(x, y, parentIndex, currentCost):
    global nodeCount
    
    nodeCount += 1
    index = nodeCount
    cost = currentCost + 1.0
    
    newNode = [(x-1,y), index, parentIndex, cost]
    Djikstra(newNode)
    
def moveU(x, y, parentIndex, currentCost):
    global nodeCount
    
    nodeCount += 1
    index = nodeCount
    cost = currentCost + 1.0
    
    newNode = [(x,y-1), index, parentIndex, cost]
    Djikstra(newNode)
    
def moveD(x, y, parentIndex, currentCost):
    global nodeCount
    
    nodeCount += 1
    index = nodeCount
    cost = currentCost + 1.0
    
    newNode = [(x,y+1), index, parentIndex, cost]
    Djikstra(newNode)
    
def moveUR(x, y, parentIndex, currentCost):
    global nodeCount
    
    nodeCount += 1
    index = nodeCount
    cost = currentCost + 1.4
    
    newNode = [(x+1,y-1), index, parentIndex, cost]
    Djikstra(newNode)
    
def moveUL(x, y, parentIndex, currentCost):
    global nodeCount
    
    nodeCount += 1
    index = nodeCount
    cost = currentCost + 1.4
    
    newNode = [(x-1,y-1), index, parentIndex, cost]
    Djikstra(newNode)
    
def moveDR(x, y, parentIndex, currentCost):
    global nodeCount
    
    nodeCount += 1
    index = nodeCount
    cost = currentCost + 1.4
    
    newNode = [(x+1,y+1), index, parentIndex, cost]
    Djikstra(newNode)
    
def moveDL(x, y, parentIndex, currentCost):
    global nodeCount
    
    nodeCount += 1
    index = nodeCount
    cost = currentCost + 1.4
    
    newNode = [(x-1,y+1), index, parentIndex, cost]
    Djikstra(newNode)

## Backtracking

def backtrack(node, nodePosition):
    global solutions
    global visited
    
    global closedList
    global closedListPoints
    
    # Visualize Loop
    while True:
        displayPlot()
        
        index = node
        i = len(closedList) - 1
        
        # Iterate through visted nodes
        while i >= 0 :
            visited.append(closedListPoints[i])
            
            # Add node to solution path if index is parent index of previous node
            if index[1] == closedList[i][0] :
                index = closedList[i]
                solutions.append(closedListPoints[i])
            i -= 1
        
        # Display visited nodes on image
        visited.reverse()
        for n in visited:
            img[n[1], n[0]] = [0,0,255]
            cv.imshow("Nodes", img)
            cv.waitKey(1)
        
        # Display solution path nodes on image
        solutions.reverse()
        for m in solutions:
            img[m[1], m[0]] = [255,255,0]
            cv.imshow("Nodes", img)
            cv.waitKey(1)
            
        cv.imshow("Nodes", img)
        cv.waitKey(15000)
        cv.destroyAllWindows()
        
        solutions.clear()
        visited.clear()


### Main Program
#####

# Get starting and ending coordinates
start = getCoordinate(True)
goal = getCoordinate(False)
print("\n\nLoading...\n\n")

# Add start nodes to first group of nodes to visit
openList.append([0, 0, 0, start[0], start[1]])

while not finish:
    
    while openList:
        currentNode = openList.popleft()
        currentNodePosition = [currentNode[3], currentNode[4]]
        
        if currentNodePosition not in closedListPoints:
            closedList.append([currentNode[0], currentNode[1], currentNode[2]])
            closedListPoints.append([currentNodePosition[0], currentNodePosition[1]])
        
        if currentNodePosition == goal:
            
            goalNode = currentNode
            goalNodePosition = currentNodePosition
            
            finish = True

        else:
            move(currentNode, currentNodePosition)
            
    # Reorder list by cost to search the lowest cost values first
    interList = list(openStage)
    for i, j in enumerate(interList):
        interList[i].append(openStagePoints[i][0])
        interList[i].append(openStagePoints[i][1])
    interList = sorted(interList, key=lambda l:l[2])
    openList = deque(interList)
    
    openStage.clear()
    openStagePoints.clear()
    
backtrack(goalNode, goalNodePosition)