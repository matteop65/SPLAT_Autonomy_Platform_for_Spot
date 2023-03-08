#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 10:46:50 2023

@author: Matteo Penlington (1924672)

This files purpose is: given a launch file path, it will start the launch file.
This file does not have a main function as it is called by SPLAT_ROS.py and SPLAT_main.py
"""

# import libraries
import rospy, roslaunch
import os

def start_launch_file(launch_file_path,msg):
    """
        This functions starts a launch file.
        returns True if it succeeded, false if error was encountered
    """
    # check that launch file path exists
    if not os.path.isfile(launch_file_path):
        raise(Exception("Could not find launch file: "+launch_file_path))
        
    
    try:
        # create launch file instance
        uuid= roslaunch.rlutil.get_or_generate_uuid(None, False)
        launch_instance = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
        launch_instance.start()
        rospy.loginfo(msg+launch_file_path)
        return True
    except Exception:
        rospy.loginfo("[ERROR] Failed to start launch file: "+launch_file_path)
        rospy.loginfo(Exception)
        return False

