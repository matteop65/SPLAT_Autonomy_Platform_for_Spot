#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  9 11:16:08 2023

@author: spot

Main script to call:
    - SPLAT_ROS
    - SPLAT_http_server
    
    
Run this script in python 2 due to ROS requiring python 2 to run. 
"""

# import libraries
import sys
import os
import multiprocessing
import subprocess
import time
import rospy

# import custom libraries
from python_launch import start_launch_file


def http_server(cur_dir, port, cam):
    path = os.path.join(cur_dir, "SPLAT_http_server.py")
    if not os.path.isfile(path):
        print('path doesn\'t exist')
    cam = 'back'
    cmd = ["/home/spot/miniconda3/envs/HTTP/bin/python3",path,'--p',str(port), '--img_pth','/home/spot/catkin_ws/src/spot/tmp/'+cam+'.jpg']
    return subprocess.call(cmd)
    
#    cam = 'left'
#    cmd = ["/home/spot/miniconda3/envs/HTTP/bin/python3",path,'--p','5002', '--img_pth','/home/spot/catkin_ws/src/spot/tmp/'+cam+'.jpg']
#    subprocess.call(cmd)
#
#    cam = 'right'
#    cmd = ["/home/spot/miniconda3/envs/HTTP/bin/python3",path,'--p','5003', '--img_pth','/home/spot/catkin_ws/src/spot/tmp/'+cam+'.jpg']
#    subprocess.call(cmd)
#
#    cam = 'frontright'
#    cmd = ["/home/spot/miniconda3/envs/HTTP/bin/python3",path,'--p','5004', '--img_pth','/home/spot/catkin_ws/src/spot/tmp/'+cam+'.jpg']
#    subprocess.call(cmd)
#    
#    cam = 'frontleft'
#    cmd = ["/home/spot/miniconda3/envs/HTTP/bin/python3",path,'--p','5005', '--img_pth','/home/spot/catkin_ws/src/spot/tmp/'+cam+'.jpg']
#    subprocess.call(cmd)

#    return

def ros(cur_dir):
    path = os.path.join(cur_dir, "SPLAT_ROS.py")
    
    os.system(path)
#    cmd = ["/usr/bin/env",cur_dir ]
#    return subprocess.call(cmd)

def find_path(cur_dir):
    path = os.path.join(cur_dir, "/home/spot/catkin_ws/src/spot/SPOT_Inspection/AutonomousNavigation/find_path.py")
    cmd = ["/home/spot/miniconda3/envs/AutonomousNavigation/bin/python3",path ]
    return subprocess.call(cmd)
    

if __name__ == '__main__':
    # first start roscore and run spot_driver
#    roscore = subprocess.Popen('roscore')
#    time.sleep(10) # roscore needs a few seconds to launch

#    # then start spot driver
#    spot_driver_file_path = "/home/spot/catkin_ws/src/spot_ros/spot_driver/launch/driver.launch" # path to launch file
#    msg = "[INFO] Starting spot driver" # message to display whilst starting launch files
#    if not start_launch_file(spot_driver_file_path,msg): # if launch was not successful, raise exception
#        raise(Exception("[FATAL] Could not start spot driver"))
#    else:
#        rospy.loginfo("[SUCCESS] Successfully started spot driver")
    
    
    # initialise multiprocessing
    proc_count = multiprocessing.cpu_count()     # get cpu count
    pool = multiprocessing.Pool(processes=proc_count) # create pool
    
    # scripts to run
    process_list = ['SPLAT_ROS.py','SPLAT_http_server.py',"/home/spot/catkin_ws/src/spot/SPOT_Inspection/AutonomousNavigation/find_path.py"]
    threads =[]
    
    # current directory
    cur_dir = os.getcwd()
  
    # setup multipprocessing of scripts
    for process in process_list:
        if process == "SPLAT_ROS.py":
#            p = multiprocessing.Process(target=ros, args=[cur_dir])
#            p.start()

#            print p, p.is_alive()
            pass

#            threads.append(p)
        elif process == "SPLAT_http_server.py":
            p = multiprocessing.Process(target = http_server, args=[cur_dir, 5001, 'back'])
            p.start()
            threads.append(p)
#        elif process == "/home/spot/catkin_ws/src/spot/SPOT_Inspection/AutonomousNavigation/find_path.py":
#            p = multiprocessing.Process(target = find_path, args=[cur_dir])
#            p.start()
#            threads.append(p)
#            
            
    
