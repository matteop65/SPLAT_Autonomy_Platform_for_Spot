#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  9 11:51:24 2023

@author: spot

Run object detection
"""

import cv2
import warnings 

import requests
import numpy as np
import subprocess



if __name__ == "__main__":
    url = "http://localhost:5001/video_feed"
    # img_resp = requests.get(url)
    # img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    # imgOriginalScene = cv2.imdecode(img_arr, -1)
    # cv2.imshow("IPcamera", imgOriginalScene)
    
    
    warnings.filterwarnings('ignore')    
    
    cap = cv2.VideoCapture(url)
    
    
    while True:
        check, frame = cap.read()
        # print(frame)
        # image = cv2.imdecode(frame, cv2.IMREAD_COLOR)
    
        subprocess.call(["/home/spot/miniconda3/envs/spotv1/bin/python3", "ObjectDetection/yolov7/detect.py","--weights","yolov7/yolov7-tiny.pt","--source",url, "--conf", "0.4","--nosave"])

        cv2.imshow('video', frame)
        key = cv2.waitKey(1)
        if key == 27:
            break
    
    
    cap.release()
    cv2.destroyAllWindows()