"""
    Given a current and goal location, this script finds the unobstructured path to the goal.

    It is based off RRT with manhattan distance, with a few adjustments.    

    This one makes the point inaccessible if it tries to get there too many times and fails
"""

import numpy as np
import pygame
import random
from decision_making import findClearance
from math import *
from ros_classes import Node


def findPath(screen, c_obstacle, c_explored, nodeInfo, XDIM, YDIM, spotPoint, goalPoint, pointRadius,visualise_rrt_map):
    NodeClearenceRadius = nodeInfo[1]

    nodeCount = 0
    maxNodes = nodeInfo[0]
    nodeDelta = nodeInfo[1]
    nodes = nodeInfo[2]
    nodeCount = nodeCount+1
    inaccessibleThreshold = maxNodes

    pygame.display.set_caption('Performing RRT')

    notThere = True
    while notThere: # whilst path is not found
        if nodeCount < maxNodes:
            foundNext = False
            while foundNext == False:
                rand = get_random_clear(screen, c_obstacle,XDIM,YDIM)
                nodeCount = nodeCount+1
                # if nodeCount % 500: print('[INFO]Nodecount:{}'.format(nodeCount))
                if nodeCount > inaccessibleThreshold: # too many tries; inaccessible
                    #print('tries over limit')
                    break # breaks out of the while loop while foundNext is still false 
                parentNode = nodes[0]
                for p in nodes:
                    #if manhattan_dist(p.point, goalPoint.point) - dist(p.point,rand) <= manhattan_dist(parentNode.point, goalPoint.point) - dist(parentNode.point,rand):
                    if manhattan_dist(p.point, goalPoint.point) <= manhattan_dist(parentNode.point, goalPoint.point):
                        newPoint = step_from_to(p.point,rand,nodeDelta)
                        #if collides(screen,c_explored,newPoint) == True:
                        #    print('collides with explored')
                        #else:
                        #    print('doesnt collide')
                        if collides(screen, c_obstacle,newPoint) == False and findClearance(spotPoint,newPoint,screen,c_obstacle) == True:
                            parentNode = p
                            foundNext = True

            if foundNext==True:
                newnode = step_from_to(parentNode.point,rand,nodeDelta)
                nodes.append(Node(newnode, parentNode, dist(newnode, parentNode.point) + parentNode.cost))

                # visualise the rrt map
                if visualise_rrt_map:
                    pygame.draw.line(screen,(0,180,105),parentNode.point,newnode)
                    # screen.blit(surface, (0,0))
                    pygame.display.flip()
                
                if point_circle_collision(newnode, goalPoint.point, NodeClearenceRadius):
                    goalNodeArray = nodes[len(nodes)-1]
                    notThere = False
                    return True, goalNodeArray
            
            else:
                #print("Ran out of nodes or tries")
                print('[INFO] Max node count reached: {}'.format(nodeCount))
                return False, None
        else:
            print('[INFO] Max node count reached: {}'.format(nodeCount))
            return False, None
    





def dist(p1,p2):
	return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))
	
def manhattan_dist(p1, p2):
	return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

def point_circle_collision(p1, p2, radius):
    distance = dist(p1,p2)
    if (distance <= radius):
        return True
    return False

def step_from_to(p1,p2,delta):
    if manhattan_dist(p1,p2) < delta:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + delta*cos(theta), p1[1] + delta*sin(theta)


def dist_to_nearest_pxl(im, c_obstacle, point):
    """
        given a point, this will find the coordinates of the nearest pixel of a certain colour
    """
    # d0 = np.logical_and(im>100, im<150)
    # imgdata = pygame.surfarray.array3d(im)
    # print(imgdata[point[0],point[1]])
    #print(im)
    i = 1
    h = 1
    nearest_obstacle = False
    while not nearest_obstacle:
        start = -i
        stop = i
        step = 1
        h = np.arange(start, stop, step)
        for x in h:
            for y in h:
                if abs(x) == i or abs(y) == i:
                    # print(f'i: {i}, x:{x},y:{y}')
                    if collides(im, c_obstacle,[point[0]+x,point[1]+y]):
                        # print(f'collision at: {x,y}') 
                        nearest_obstacle = True
        if i == 50:
            break
        i = i+1



def collides(screen, c_obstacle, point):
    """
        If point p is on obstacle, then consider collision
        Returns false if no collision
    """
    # RGB values for point p
    c = screen.get_at([int(point[0]), int(point[1])])
    # c = pygame.Surface.get_at(map, (int(point[0]), int(point[1]))) 
    R = c[0]
    G = c[1]
    B = c[2]
    if R == c_obstacle[0] and G == c_obstacle[1] and B == c_obstacle[2]:
        return True
    else:
        return False


def get_random_clear(screen, c_obstacle,XDIM, YDIM):
    while True:
        # get a random point
        p = [random.random()*XDIM, random.random()*YDIM]

        width = pygame.display.Info().current_w
        height = pygame.display.Info().current_h

        if p[0] > 0 and p[0] < width and p[1] > 0 and p[1] < height:
            # if it doesn't collide
            # print(p[0],p[1])
            # print()
            # print()
            # for point in p:
            # c = pygame.Surface.get_at(map, (int(p[0]), int(p[1]))) 
            c = screen.get_at([int(p[0]), int(p[1])])

            R = c[0]
            G = c[1]
            B = c[2]
            if R != c_obstacle[0] and G != c_obstacle[1] and B != c_obstacle[2]:
                return p
        # noCollision = collides(rectObs,p)
        # if noCollision == False:
        #     return p