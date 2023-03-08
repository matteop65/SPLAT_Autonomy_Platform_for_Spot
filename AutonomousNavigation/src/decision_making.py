import pygame
import numpy as np
from math import atan2

from ros_classes import Node


def findGoal(spotPose, screen, quadrant, c_unexplored, c_obstacle, c_spot, pointRadius):
    """
        This is the decision making module
        Given a start point, a map and obstacles, it will find a goal Point
    """
    goalFound = False
    goal = spotPose # initialise
    distanceFromEdge = 10
    width = pygame.display.Info().current_w
    height = pygame.display.Info().current_h
    pi = 3.14159165

    i = 1 # layer number
    while goalFound == False:
        # create the array to pass through, this depends on layer number
        start = -i
        stop = i+1 # the stop value is not included in array
        points_array = np.arange(start, stop, 1)

        # points_array = sorted(points_array, key=lambda x: abs(x))
        # points_array = [-x if x < 0 else x for x in points_array]

        for u in points_array:
            for v in points_array:
                if abs(u) == i or abs(v) == i: # make sure points lie on the correct layer
                    # if spotPose.point[0]+u < width-distanceFromEdge and spotPose.point[0]+u > distanceFromEdge and spotPose.point[1]+v < height-distanceFromEdge and spotPose.point[1]+v > distanceFromEdge: # make sure points lie in image
                    #     if quadrant == 1 and u > 0 and v > 0:
                    #         goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)
                    #     elif quadrant == 2 and u < 0 and v > 0:
                    #         goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)
                    #     elif quadrant == 3 and u < 0 and v < 0:
                    #         goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)
                    #     elif quadrant == 4 and u > 0 and v < 0:
                    #         goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)

                    if spotPose.point[0]+u < width-distanceFromEdge and spotPose.point[0]+u > distanceFromEdge and spotPose.point[1]+v < height-distanceFromEdge and spotPose.point[1]+v > distanceFromEdge: # make sure points lie in image
                        if quadrant == 1 and atan2(v,u) < pi /4 and atan2(v,u) > 0:
                            goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)
                            # print('[INFO] Found potential goal in quadrant 1')
                        elif quadrant == 2 and atan2(v,u) >= pi/4 and atan2(v,u) < pi/2:
                            goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)
                        elif quadrant == 3 and atan2(v,u) >= pi/2 and atan2(v,u) < 3*pi/4:
                            goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)
                        elif quadrant == 4 and atan2(v,u) >= 3*pi/4 and atan2(v,u) < pi:
                            goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)
                        elif quadrant == 5 and atan2(v,u) >= pi and atan2(v,u) < 5*(pi/4) :
                            goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)
                        elif quadrant == 6 and atan2(v,u) >= 5*pi/4 and atan2(v,u) < 3*(pi/2) :
                            goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)
                        elif quadrant == 7 and atan2(v,u) >= 3*pi/2 and atan2(v,u) < 7*(pi/4) :
                            goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)
                        elif quadrant == 8 and atan2(v,u) >= 7*pi/4:
                            goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)
                        else: 
                            goalFound, goalPoint = process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle)

                        if goalFound:
                            return goalFound, goalPoint
        if i > width or i > height:
            return False, None
        i = i+1

    return False, None


def process_goal(u,v,spotPose, screen, c_unexplored, c_obstacle):
    """
        Determines if potential goal is possible
    """
    goal = [int(spotPose.point[0]+u), int(spotPose.point[1]+v)] # goal point
    goalColourTemp = screen.get_at(goal) # foreground pixel colour at goal point
    backgroundGoalColourTemp = screen.get_at(goal) # background pixel colour at goal point

    if np.sqrt(u*u + v*v) > 15:
        # if pixel colour at goal matches unexplored colour, and there's no obstacle there, then goal point is found
        if int(goalColourTemp[0]) == int(c_unexplored[0]) and int(goalColourTemp[1]) == int(c_unexplored[1]) and int(goalColourTemp[2]) == int(c_unexplored[2]):
            # print(f'backgroundGoalColourTemp: {backgroundGoalColourTemp}')
            # print(f'c_obstacle: {c_obstacle}')
            if int(backgroundGoalColourTemp[0]) != int(c_obstacle[0]):
                if int(backgroundGoalColourTemp[1]) != int(c_obstacle[1]):
                    if int(backgroundGoalColourTemp[2]) != int(c_obstacle[2]):
                        clear = findClearance(spotPose.point,goal,screen,c_obstacle) # finds clearance
                        if clear == True:
                            goalFound=True
                            return goalFound, Node( (goal[0],goal[1]),None)
                        else:
                            print('[INFO] Goal did not meet clearence criteria')

    return False, None
                    

def findClearance(spotPoint,goalPoint,screen,obstacle_colour):
    """
        This is the obstacle clearange module. 
        Given a point, the map (screen), and the obstacle colour, it'll find whether there's few enough
        obstacles to go there safety.
        By John u1935805 :)
    """
    midClear = False # variable to return to say if we're clear at the midpoint or not
    goalClear = False # variable to return to say if we're clear at the goalpoint or not
    clearenceThreshold = 0.5

    [width, height] = screen.get_size() # gets the width and height of the map 
    radius = 20 # 17 pixel search radius
    searchRange = ((2*radius) + 1)^2 # = ((2*radius) + 1)^2: the square in which we're searching for obstacles
    nearRadius = 6 # nearby pixel search radius
    nearRange = 20
    
    nearRangeArea = int(nearRange * nearRange * 3.14159)
    searchRangeArea = searchRange # int(searchRange * searchRange * 3.14159)

    # Make sure goal & spot points are integers because sometimes they can be funny
    if isinstance(goalPoint, np.float64):
        goalPoint = goalPoint.astype(int)
    
    if isinstance(spotPoint, np.float64):
        spotPoint = spotPoint.astype(int)

    # calculate the midpoint:
    midPoint = [0,0] # initialise
    midPoint[0] = (spotPoint[0]+goalPoint[0])/2 # halfway between current & goal x values
    midPoint[1] = (spotPoint[1]+goalPoint[1])/2 # halfway between current & goal y values

    if isinstance(midPoint, np.float64):
        midPoint = midPoint.astype(int)

    # for the goal point:
    g = 1 # goal layer number
    gNearObsCount = 0 # number of nearby obstacles found
    gFarObsCount = 0 # number of far away obstacles found
    gGoalDelta=1 # how far out from the point we're searching right now 

    # for the midpoint:
    m = 1 # midpoint layer number
    mNearObsCount = 0 # number of nearby obstacles found
    mFarObsCount = 0 # number of far away obstacles found
    mGoalDelta=1 # how far out from the point we're searching right now
    

    # Goal clearance search:
    while g<radius: # runs until we've searched everything in the radius
        
        start = -g
        stop = g+1
        step = g
        delta_array = np.arange(start, stop, step) # array of all the points within a range

        for u in delta_array: # checks in the x direction
            for v in delta_array: # checks in the y direction
                if abs(u) == gGoalDelta or abs(v) == gGoalDelta: # looks at the points at range goalDelta
                    pointTemp = [(goalPoint[0]+u), (goalPoint[1]+v)] # gets the coordinates of that point
                    if pointTemp[0]>=0 and pointTemp[1]>=0 and pointTemp[0] < width and pointTemp[0] < height and pointTemp[1] < width and pointTemp[1] < height: # makes sure the point's not out of bounds
                        #print(pointTemp)
                        goalColourTemp = screen.get_at([int(pointTemp[0]),int(pointTemp[1])]) # checks the colour at that point
                        # checks if there's an obstacle at that point, ie the colours match up
                        if int(goalColourTemp[0]) == int(obstacle_colour[0]) and int(goalColourTemp[1]) == int(obstacle_colour[1]) and int(goalColourTemp[2]) == int(obstacle_colour[2]):
                            if gGoalDelta < nearRange:
                                gNearObsCount = gNearObsCount + 1 # near obstacle found! record it
                            else:
                                gFarObsCount = gFarObsCount + 1 # far obstacle found! record it
        gGoalDelta = gGoalDelta + 1 # we've checked every square in the range goalDelta, so expand the range
        g = g+1 # increment the counter

    #print('Search range:', searchRange) # print for debugging
    #print('0.2 * searchRange:', (searchRange*0.2)) # print for debugging
    #print('obsCount: ', obsCount) # print for debugging
    if gFarObsCount > (clearenceThreshold*searchRangeArea): #or nearObsCount > (0.2*searchRange): # Obstacles can't occupy more than 10% of the search range
        #print('farObs!') # print for debugging
        goalClear = False
    elif gNearObsCount>(clearenceThreshold*nearRangeArea):
        #print('nearObs!')
        goalClear = False
    else:
        #print('Clear!') # print for debugging
        goalClear = True



    # Midpoint clearance search:
    while m<radius: # runs until we've searched everything in the radius
        
        start = -m
        stop = m+1
        step = m
        delta_array = np.arange(start, stop, step) # array of all the points within a range

        for u in delta_array: # checks in the x direction
            for v in delta_array: # checks in the y direction
                if abs(u) == mGoalDelta or abs(v) == mGoalDelta: # looks at the points at range goalDelta
                    pointTemp = [(midPoint[0]+u), (midPoint[1]+v)] # gets the coordinates of that point
                    if isinstance(pointTemp, np.float64):
                        pointTemp = pointTemp.astype(int)
                    if pointTemp[0]>=0 and pointTemp[1]>=0 and pointTemp[0] < width and pointTemp[0] < height and pointTemp[1] < width and pointTemp[1] < height: # makes sure the point's not out of bounds
                        midColourTemp = screen.get_at([int(pointTemp[0]),int(pointTemp[1])]) # checks the colour at that point
                        # checks if there's an obstacle at that point, ie the colours match up
                        if int(midColourTemp[0]) == int(obstacle_colour[0]) and int(midColourTemp[1]) == int(obstacle_colour[1]) and int(midColourTemp[2]) == int(obstacle_colour[2]):
                            if mGoalDelta < nearRange:
                                mNearObsCount = mNearObsCount + 1 # near obstacle found! record it
                            else:
                                mFarObsCount = mFarObsCount + 1 # far obstacle found! record it
        mGoalDelta = mGoalDelta + 1 # we've checked every square in the range goalDelta, so expand the range
        m = m+1 # increment the counter

    #print('Search range:', searchRange) # print for debugging
    #print('0.2 * searchRange:', (searchRange*0.2)) # print for debugging
    #print('obsCount: ', obsCount) # print for debugging
    if mFarObsCount > (clearenceThreshold*searchRangeArea): #or nearObsCount > (0.2*searchRange): # Obstacles can't occupy more than 10% of the search range
        #print('farObs!') # print for debugging
        midClear = False
    elif mNearObsCount>(clearenceThreshold*nearRangeArea):
        #print('nearObs!')
        midClear = False
    else:
        #print('Clear!') # print for debugging
        midClear = True


    # Are both clear?
    if goalClear==True and midClear==True:
        allClear = True
        #print('all clear')
        return allClear
    else:
        allClear = False
        #print('all not clear')
        #print('goalclear:', goalClear)
        #print('midclear:',midClear)
        return allClear