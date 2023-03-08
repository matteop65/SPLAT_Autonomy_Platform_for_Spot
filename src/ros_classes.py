"""
    This script creates the class to find Spots current position in world coordinates
"""

# import ros packages
import rospy, roslaunch
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Point, Twist, Pose
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

# import other packages
import time
from datetime import datetime
from math import atan2
import numpy as np
import os
import csv
import cv2

class Node(object):
    """
        Node class used for navigation
    """
    def __init__(self, point, parent, cost =0):
        super(Node, self).__init__()
        self.point = point # a tuple (x,y)
        self.parent = parent
        self.cost = cost

class image_loader:
    # load image from greyscale cameras on spot
    def __init__(self, camera, data_dir):
        # initialise subscriber topics
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/spot/camera/"+camera+"/image",Image, self.img_callback)
        self.camera_name = camera
        self.data_save = data_dir
        
        
    def img_callback(self, data):
        # convert ROS Image to CV2 Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Rotate image according to which image it is   
        if self.camera_name == "right":
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
        elif self.camera_name == "frontleft":
            cv_image = self.rotate(cv_image, -78, None, 1.0)
        elif self.camera_name == "frontright":
            cv_image = self.rotate(cv_image, -102, None, 1.0)  
        
        
        # save image for separate file to upload to http server
        unix_time = int(time.time())
        folder_path = os.path.join(self.data_save, self.camera_name)
        if not os.path.isdir(folder_path):
            os.mkdir(folder_path)
        cv2.imwrite(folder_path+'/'+str(unix_time)+".jpg",cv_image)
        cv2.imwrite("tmp/"+self.camera_name+".jpg",cv_image)


    def rotate(self, image, angle, center = None, scale = 1.0):
        """
            Rotate image according to angle in degrees. 
        """
        (h, w) = image.shape[:2]
    
        if center is None:
            center = (w / 2, h / 2)
    
        # Perform the rotation
        M = cv2.getRotationMatrix2D(center, angle, scale)
        rotated = cv2.warpAffine(image, M, (w, h))
    
        return rotated


class spot_pose:
    """
        returns spots pose
    """    
    # x = 0
    # y = 0
    def __init__(self, topic, save_folder):
        self.pose_sub = rospy.Subscriber(topic,Odometry, self.pose)
        self.save_folder = save_folder
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.save_folder = save_folder
        
    def pose(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        
        spot_rotation_q = data.pose.pose.orientation

        # convert from quaternion to euler. Only interested in Yaw for fixed frame simple motion
        self.yaw = euler_from_quaternion(spot_rotation_q)


        unix_time = time.time()
        datetime_time = datetime.utcfromtimestamp(unix_time).strftime('%Y-%m-%d_%H-%M-%S')
        
        # save spot position in world coordinates
        csv_info_file_name = os.path.join(self.save_folder, 'spot_position.csv')
        with open(csv_info_file_name, mode='a') as csv_file:
            # Create a CSV writer object
            writer = csv.writer(csv_file)  

            # Write the header row if the file is empty
            if csv_file.tell() == 0:
                writer.writerow(['UNIX_time', 'datetime', 'Position_x(m)', 'Position_y(m)','Orientation_yaw'])

            writer.writerow([unix_time, datetime_time, self.x, self.y, self.yaw])
# #        if not os.path.
        return # [self.x, self.y]
    

class occupancy_grid:
    """
        Subscribes to topic to get all occupancy grid related data    
    """
    def __init__(self, topic, grid_save_path, grid_info_save_path):
        self.grid_sub = rospy.Subscriber(topic, OccupancyGrid, self.prepare_data)

        self.grid_data = [] # the occupancy grid data in array form
        self.origin = [] # origin of map in world coordinates
        self.height = 0 # height of map
        self.width = 0 # widthof map
        self.resolution = 0.05 # m/pixel resolution (around 0.05 for RealSense cameras)

        # save directories
        self.grid_save_path = grid_save_path
        self.grid_info_save_path = grid_info_save_path

    def prepare_data(self, data):
        """
            Gets relevant occupancy grid data
        """
        self.grid_data = data.data
        self.origin = [data.info.origin.position.x, data.info.origin.position.y]
        self.height = data.info.height
        self.width = data.info.width
        self.resolution = data.info.resolution
        
        unix_time= time.time()
        datetime_time = datetime.utcfromtimestamp(unix_time).strftime('%Y-%m-%d_%H-%M-%S')
        
        # save grid data
        csv_file_name = os.path.join(self.grid_save_path, 'grid_'+str(unix_time)+'.csv')
        np.savetxt(csv_file_name, np.array(self.grid_data), delimiter=',')

        # save grid info
        csv_info_file_name = os.path.join(self.grid_info_save_path, 'grid_info.csv')
        with open(csv_info_file_name, mode='a') as csv_file:
            # Create a CSV writer object
            writer = csv.writer(csv_file)  

            # Write the header row if the file is empty
            if csv_file.tell() == 0:
                writer.writerow(['UNIX time', 'datetime', 'Origin_x', 'Origin_y', 'Height', 'Width', 'Resolution'])

            writer.writerow([unix_time, datetime_time, data.info.origin.position.x, data.info.origin.position.y, self.height, self.width, self.resolution])



class move_spot:
    """
        Moves spot to the dedicated location/s
    """
    def __init__(self, pubTopic):
        # initialise publisher topic
        self.cmd_vel_pub = rospy.Publisher(pubTopic,Twist, queue_size=1)

        self.speed = Twist()

        # There are small error induced due to sensor accuracy and updates. 
        # So this is used to quantify when Spot is close enough to required orientation.
        self.angle_error = 0.1

    def motion_controller(self, spotPose, node_list):
        # print('spotPose: {} {} {}'.format(spotPose.x, spotPose.y, spotPose.yaw))
        i = 1
        pub = self.cmd_vel_pub

        for node in node_list:

            # print('node: {}'.format(node))
            # calculate distance and angle to goal position
            dx = node[0] - spotPose.x
            dy = node[1] - spotPose.y
            dist_to_node = np.sqrt(dx*dx + dy*dy)
            angle_to_node = atan2(dy,dx)

            print('\n[INFO] Node {} out of {}'.format(i,len(node_list)))
            print('[INFO] Distance to goal: {} m'.format(dist_to_node))
            print('[INFO] Angle to node: {}'.format(angle_to_node))

            motion_counter = 0                
            start_time = time.time()
            while dist_to_node >= 0.5:
                cur_time = time.time()
                if abs(start_time - cur_time) > 10:
                    break
                # if abs(angle_to_node - spotPose.yaw) > self.angle_error:
                # counter = 0

                while abs(angle_to_node - spotPose.yaw) > self.angle_error:
                    # print information only once
                    # if counter == 0:
                    #     # print('[INFO] Rotating Spot')
                    # counter = counter + 1
                    motion_counter = 0                


                    # rotate spot to required angle
                    self.speed.linear.x = 0.0 # no linear velocity

                    if angle_to_node - spotPose.yaw > 0:
                        self.speed.angular.z = 0.3 # clockwise
                    else:
                        self.speed.angular.z = -0.3


                    dx = node[0] - spotPose.x
                    dy = node[1] - spotPose.y
                    dist_to_node = np.sqrt(dx*dx + dy*dy)
                    angle_to_node = atan2(dy,dx)
                    
                    pub.publish(self.speed)

                self.speed.linear.x = 0.5
                self.speed.angular.z = 0

                pub.publish(self.speed)

                dx = node[0] - spotPose.x
                dy = node[1] - spotPose.y
                dist_to_node = np.sqrt(dx*dx + dy*dy)
                angle_to_node = atan2(dy,dx)


            self.speed.linear.x = 0
            self.speed.angular.z =0
            # publish the information to spot
            pub.publish(self.speed)
            i = i+1

    def rotate_360(self, spotPose):
        time_unix = time.time()
        current_time = time.time()

        pub = self.cmd_vel_pub

        t = (3.14159265 * 2) / 0.5

        while abs(current_time - time_unix) < t:
            self.speed.angular.z = 0.5
            self.speed.linear.x = 0
            pub.publish(self.speed)
            current_time = time.time()



        self.speed.linear.x = 0
        self.speed.angular.z =0
        pub.publish(self.speed)





            

def euler_from_quaternion(rot_q):
    """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
    """
    x = rot_q.x
    y = rot_q.y
    z = rot_q.z
    w = rot_q.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)

    return yaw_z # in radians

