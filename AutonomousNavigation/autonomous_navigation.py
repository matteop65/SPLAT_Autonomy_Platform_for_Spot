
# import ros packages
import rospy, roslaunch
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Point, Twist, Pose


# import general packages
import cv2 # to process image/camera data
import subprocess # to run different scripts in different versions of python
import numpy as np
from math import atan2, sqrt
import os
import time
from datetime import datetime
import pygame # for mapping

# import custom pacakges
from src.ros_classes import *
from src.decision_making import findGoal
from src.rrt_manhattan_distance import findPath


# some colours that we will use to distinguish parts of the map
white = 255, 255, 255
black = 25, 25, 25
red = 255, 0, 0
green = 0, 255, 0
blue = 0, 0, 255
cyan = 0,180,105
orange = 255, 95, 31




def update_map(grid, spot_position, c_unexplored, c_explored, c_obstacle, c_spot,pointRadius):
    # get origin of map for pixel (0,0) in meters 
    origin_x = grid.origin[0] # 
    origin_y = grid.origin[1] # 
    
    # pixel height and width of map. Height and width are switched purposely to decode data
    height = grid.width # 
    width = grid.height # 

    # get spot position
    spot_x = spot_position.x # 
    spot_y = spot_position.y # 
    
    # conversion for m/pixel
    resolution = grid.resolution# 

    # Set up the drawing window
    screen = pygame.display.set_mode([width, height])

    # Fill the background 
    screen.fill(c_unexplored)

    # resize the grid_data array to be in width,height
    data = np.resize(grid.grid_data, (width, height))

    # draw the pixels on the map
    for row_element in range(width):
        for height_element in range(height):
            if data[row_element, height_element] == -1:
                pygame.draw.circle(screen, c_unexplored, (row_element, height_element),pointRadius)
            elif data[row_element, height_element] == 0:
                pygame.draw.circle(screen, c_explored, (row_element, height_element),pointRadius)
            elif data[row_element, height_element] == 100:
                pygame.draw.circle(screen, c_obstacle, (row_element, height_element),5*pointRadius)

    ###################
    # Put spot position on map
    ###################
    # map is flipped 180 degrees due to spot's fixed frame coordinates not matching natural direction. 
    # hence coordinates (0,0) on pygame match with the bottom left of the map in spots (x,y) 
    # (but spots x,y frame is flipped by 90 degrees)
    dy =  abs((spot_x - origin_x)/resolution) # find the distance x between map origin and spot in fixed frame
    dx = abs((spot_y - origin_y)/resolution) # find the distance y between map origin and spot in fixed frame
    pygame.draw.circle(screen, c_spot, (dx,dy),2*pointRadius)

    # update map
    pygame.display.flip()

    return screen, [dx, dy]

def main():
    pointRadius = 1
    maxNodes = 1000
    nodeDelta = 6


    # define appropriate colours
    c_unexplored = orange
    c_obstacle = black
    c_explored = white
    c_spot = red
    c_goal = blue
    
    # initialise pygame
    pygame.init()

    ###################
    # Create directories
    ###################

    # create main data directory
    cur_dir = os.getcwd()
    DATA_dir = os.path.join(cur_dir, "DATA")
    if not os.path.isdir(DATA_dir):
        os.mkdir(DATA_dir)

    # create directory specific to runtime
    unix_time= time.time()
    datetime_time = datetime.utcfromtimestamp(unix_time).strftime('%Y-%m-%d_%H-%M-%S')
    save_data_dir= os.path.join(DATA_dir, datetime_time)
    if not os.path.isdir(save_data_dir):
        os.mkdir(save_data_dir)

    # create sub directories
    grid_save_path = os.path.join(save_data_dir,'occupancy_grid')
    grid_info_save_path = os.path.join(save_data_dir,'grid_info')
    pygame_map_save_path = os.path.join(save_data_dir, 'pygame_maps')
    if not os.path.isdir(grid_save_path):
        os.mkdir(grid_save_path)
    if not os.path.isdir(grid_info_save_path):
        os.mkdir(grid_info_save_path)
    if not os.path.isdir(pygame_map_save_path):
        os.mkdir(pygame_map_save_path)


    ###################
    # initialise ROS
    ###################
    rospy.init_node("Autonomous_Navigation",anonymous=True)
    rospy.loginfo("Initiating SPLAT ROS node")
    r = rospy.Rate(4)

    # initialise ros classes
    spot_position = spot_pose("/spot/odometry", save_data_dir)
    grid = occupancy_grid("/rtabmap/grid_map", grid_save_path, grid_info_save_path)
    moving_spot = move_spot("/cmd_vel")
    time.sleep(1) # give time to info to load

    # initialise data gathering of the images
    img_disp_right = image_loader("right",save_data_dir)
    img_disp_left = image_loader("left",save_data_dir)
    img_disp_back = image_loader('back',save_data_dir)
    img_disp_frontright = image_loader("frontright",save_data_dir)
    img_disp_frontleft = image_loader("frontleft",save_data_dir)


    # initialise variables 
    status = 'findGoal' # status
    quadrant = 0 # search quadrant. 0 = search for closest pixel 
    rotateCounter = 0 # rotation counter. every 3 goal points spot will rotate
    xNodes = [] # the variable with the x position of the RRT nodes
    yNodes = [] # the variable with the y position of the RRT nodes
    iteration = 1 # used to name images

    print('[INFO] spotPose (x,y,yaw): {} {} {}'.format(spot_position.x, spot_position.y, spot_position.yaw))

    # while ros is running
    while not rospy.is_shutdown():

        # first update map
        screen, spotPosePxl = update_map(grid, spot_position, c_unexplored, c_explored, c_obstacle,c_spot, pointRadius)

        # if a goalPoint variable exists, then update map now. This ensures that Spot's goal point is always visible to the operator. 
        if 'goalPointPxl' in locals():
            # if the findGoal function fails. the goal point variable exists but has value None. Hense a try and except statement is implement as a way around this. 
            try:
                pygame.draw.circle(screen, c_goal, (goalPointPxl.point[0],goalPointPxl.point[1]), 2*pointRadius)
                pygame.display.flip()
            except:
                pass                


        if status == "findGoal": # find goal point
            print('[INFO] Finding Goal...')

            if quadrant > 8: # if the quadrant is above 8, then the program is done
                raise(Exception("[INFO] Could not find goal within the 8 quadrants. Thus, mapping is concluded. ")) 

            # convert spots position in pixels to class node as used by RRT
            spotPose = Node( (spotPosePxl[0], spotPosePxl[1]), None)

            # find goal
            success, goalPointPxl = findGoal(spotPose, screen, quadrant, c_unexplored, c_obstacle, c_spot, pointRadius)

            if success: # if goal is successfully found
                # transform from pixel to world coordinates
                goalPoint_x = (goalPointPxl.point[1] * grid.resolution) + grid.origin[0]
                goalPoint_y = (goalPointPxl.point[0] * grid.resolution) + grid.origin[1]
                goalPoint = [goalPoint_x, goalPoint_y]

                print('\033[1;32m[SUCCESS] Goal found at: {} or in pixels {}\033[0m'.format(goalPoint, goalPointPxl.point))

                # draw goal location on map
                pygame.draw.circle(screen, c_goal, (goalPointPxl.point[0],goalPointPxl.point[1]), 2*pointRadius)
                pygame.display.flip()

                # Slow down the algorithm to allow the operator to visually inspect the goal point
                time.sleep(1)

                # set status to determine path.
                status = 'determinePath'

            else: # if finding goal was unsuccessful
                # increase quadrant search by 1 
                quadrant = quadrant+1

                print('\033[1;31m[ERROR] Goal not found, looking again using quadrant {}\033[0m'.format(quadrant))

                # rotate spot 360 to update occupancy map. If goal was not found due to poor data, this should fix it. 
                status = 'rotateSpot'


        elif status == 'determinePath': # find path between Spots current position and goal location. 
            print('[INFO] Finding Path...')

            # initialise a node array used by RRT to store path nodes. 
            nodeArray = []
            nodeArray.append(spotPose) # add spot position as the first node. 

            # an array for pass to the findPath function
            nodeInfo = [maxNodes, nodeDelta, nodeArray]

            # run the findPath function
            success, path = findPath(screen, c_obstacle, c_explored, nodeInfo, grid.height, grid.width, spotPose.point, goalPointPxl, pointRadius,1)

            if success: # if path is successfully found, apply filtering to the nodes

                print('\033[1;32m[SUCCESS] Path found\033[0m')

                ###################################
                # get the list of points for path #
                ###################################
                # nodes in path are stored in order: last --> first. However, we want first -->. Additionally, the path does not include the initial or goal points and thus need addding. 
                # xNodes, yNodes represent the nodes first --> last including start and end point. 
                # xNodes_mid, yNodes_mid represent the nodes last --> first (before being flipped) excluding start and end points
                # xNodesFiltered, yNodesFiltered represent the nodes first --> last with filtering to increase the distance between nodes to a desired threshold. 

                # add starting point
                xNodes = [spotPose.point[0]]
                yNodes = [spotPose.point[1]]

                # initialise the middle node arrays
                xNodes_mid = []
                yNodes_mid = []

                # convert the RRT path into node arrays
                while path != None:
                    xNodes_mid = np.append(xNodes_mid, np.array(path.point[0]))
                    yNodes_mid = np.append(yNodes_mid, np.array(path.point[1]))
                    path = path.parent
                
                # flip the mid arrays such that they are first --> last (excludes start and goal point)
                xNodes_mid = np.flip(xNodes_mid,axis=0)
                yNodes_mid = np.flip(yNodes_mid,axis=0)

                # then create the proper full node list in first --> last (including start point)
                for i in range(len(xNodes_mid)):
                    xNodes = np.append(xNodes, xNodes_mid[i])
                    yNodes = np.append(yNodes, yNodes_mid[i])

                # add goal point
                xNodes = np.append(xNodes, goalPointPxl.point[0])
                yNodes = np.append(yNodes, goalPointPxl.point[1])

                # generate list to keep from the node arrays xNodes and yNodes
                take_array = np.arange(0,len(xNodes), 10) # generates index list
                xNodesFiltered = np.take(xNodes, take_array)
                yNodesFiltered  = np.take(yNodes, take_array)

                # re-add goal point, as the previous function removes the last element. 
                xNodesFiltered = np.append(xNodesFiltered, goalPointPxl.point[0])
                yNodesFiltered = np.append(yNodesFiltered, goalPointPxl.point[1])

                # transform from pixel to world coordinates
                node_list = []
                for i in range(len(xNodesFiltered)):
                    x = xNodesFiltered[i] * grid.resolution + grid.origin[1]
                    y = yNodesFiltered[i] * grid.resolution + grid.origin[0]
                    node_list.append( [y,x])

                # ready to move spot
                status = 'moveSpot'

                # path was successful, thus set quadrant = 0 to search for closest unexplored area. 
                quadrant = 0

            else: # if path could not be found
                # increase quadrant
                quadrant = quadrant + 1

                print('\033[1;31m[ERROR] Path not found. Searching for new goal using quadrant {}\033[0m'.format(quadrant))
                
                # rotate spot 360 to update occupancy map. If path was not found due to poor data, this should fix it. 
                status = 'rotateSpot'


        elif status == 'moveSpot': # if status is to move spot
            print('[INFO] Moving Spot to the following goal locations')

            # call motion controller and provide it with the list of nodes in world coordinate to go to
            moving_spot.motion_controller(spot_position,node_list)

            # this is a test function used in the lab to ensure the motion controller is functioning appropriately. It is commented in when and if required. The nodes have been manually selected. 
            # moving_spot.motion_controller(spot_position, [[6.64,1.96], [0.7,2.92]]) 

            # as Spot is now and potentially reached goal location, so now monitor its location and motion. 
            status = 'MonitoringSpot'

            print('[INFO] Monitoring Spot')
            
            # take note of the time at which monitoring began
            monitoringStartTime = time.time()


        elif status == 'MonitoringSpot': # if status is to monitor spot
            # update rotation counter
            rotateCounter = rotateCounter + 1

            # note current time
            monitoringCurTime= time.time()

            # if Spot has been monitoring for more than 10 seconds, then rotate spot (with the idea of searching for new goal location).
            # this is required as occasionally inappropriate goals are selected. This leaves Spot to remain in the monitoring status as the goal point is not reached. 
            if abs(monitoringStartTime - monitoringCurTime) > 10:
                status = 'rotateSpot'
                xNodes = []
                yNodes = []
                print('\033[1;31m[INFO] Failed to get to goal. Rotating Spot\033[0m')

            # calculate distance between spot and goal position in world coordinates
            dx = spot_position.x - goalPoint[0]
            dy = spot_position.y - goalPoint[1]
            dist_to_goal = np.sqrt(dx*dx + dy*dy)

            # if distance is less than 0.5m, then goal point is reached. 0.5m is selected as it is not required that Spot arrive at the exact location
            if dist_to_goal < 0.5: 
                status = 'rotateSpot' # change status to update occupancy map

                print('\033[1;32m[SUCESS] Arrived at goal!\033[0m\n')

                # clear path for visualisation purposes
                xNodes = []
                yNodes = []


        elif status == 'rotateSpot' and (rotateCounter % 3) == 0: # only rotate sopt is rotation counter is divisible by 3 to reduce the number of time Spot is required to rotate. 
            # rotate spot by 360 to allow for map to be updated
            print('[INFO] Rotating Spot to update occupancy map')

            # motion controller to rotate spot
            moving_spot.rotate_360(spot_position) 

            # This state is reached if: Spot fails to find goal location, fails to path, fails to get to goal, or has arrived to goal. 
            # the next step in each is to search for a new goal given an updated occupancy map
            status = 'findGoal'

        else:
            # if somehow the status is not set to any of the above, set it to findGoal
            status = 'findGoal'

        # if a goal point variable exists, then draw this in map
        if 'goalPointPxl' in locals():
            # if the findGoal function fails. the goal point variable exists but has value None. Hense a try and except statement is implement as a way around this. 
            try:
                pygame.draw.circle(screen, c_goal, (goalPointPxl.point[0],goalPointPxl.point[1]), 2*pointRadius)
            except:
                pass
        
        # visualise spots location on screen
        pygame.draw.circle(screen, c_spot, (spotPosePxl[0], spotPosePxl[1]), 2*pointRadius)

        # visualise path on screen if path and xNodesFiltered variables exist
        if 'path' in locals() and 'xNodesFiltered' in locals():
            for i in range(len(xNodesFiltered)-2):
                pygame.draw.line(screen, blue, (xNodesFiltered[i],yNodesFiltered[i]), (xNodesFiltered[i+1],yNodesFiltered[i+1]))
        pygame.display.flip()

        # save the raw image data from the cameras. 
        image_filename = os.path.join(pygame_map_save_path, '{}.png'.format(iteration))
        pygame.image.save(screen, image_filename)
        iteration = iteration + 1
        
        # sleep according to set hertz above as
        r.sleep

if __name__ == "__main__":
    main()