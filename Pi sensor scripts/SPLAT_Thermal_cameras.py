# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""This example is for Raspberry Pi (Linux) only!
   It will not work on microcontrollers running CircuitPython!"""

import os
import math
import time
import cv2
import numpy as np
import pygame
import busio
import board

from scipy.interpolate import griddata

from colour import Color

import adafruit_amg88xx

cwd = os.getcwd()
print(cwd)

i2c_bus = busio.I2C(board.SCL, board.SDA)

# low range of the sensor (this will be blue on the screen)
MINTEMP = 18.0

# high range of the sensor (this will be red on the screen)
MAXTEMP = 32.0

# how many color values we can have
COLORDEPTH = 1024

# initialize the sensor
sensor = adafruit_amg88xx.AMG88XX(i2c_bus, 0x68)
print(sensor)

sensor2 = adafruit_amg88xx.AMG88XX(i2c_bus, 0x69)
print(sensor2)

# set up the screen dimensions (8x8)
screen_width = 240
screen_height = 240

# create two separate windows
Window1 = "Gray camera"
Window2 = "Yellow camera"
cv2.namedWindow(Window1)
cv2.namedWindow(Window2)


# pylint: disable=invalid-slice-index
points = [(math.floor(ix / 8), (ix % 8)) for ix in range(0, 64)]
grid_x, grid_y = np.mgrid[0:7:32j, 0:7:32j]
# pylint: enable=invalid-slice-index

# sensor is an 8x8 grid so lets do a square
height = 240
width = 240

# the list of colors we can choose from
red = Color("red")
colors = list(red.range_to(Color("indigo"), COLORDEPTH))

# create the array of colors
colors = [(int(c.red * 255), int(c.green * 255), int(c.blue * 255)) for c in colors]

displayPixelWidth = width / 30
displayPixelHeight = height / 30


# some utility functions
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# let the sensor initialize
time.sleep(0.1)

photopath = str(cwd+'/photos')

print(photopath)

# create black images for both windows
img1 = 0 * np.ones((screen_height, screen_width, 3), dtype=np.uint8)
img2 = 0 * np.ones((screen_height, screen_width, 3), dtype=np.uint8)

# run the display loop
while True:
    # display images in both windows
    cv2.imshow(Window1, img1)
    cv2.imshow(Window2, img2)

    # exit loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

############################open cv attempt

    # read the pixels
    pixels = []

    for row in sensor.pixels:
        pixels = pixels + row
    pixels = [map_value(p, MINTEMP, MAXTEMP, 0, COLORDEPTH - 1) for p in pixels]

    # perform interpolation
    bicubic = griddata(points, pixels, (grid_x, grid_y), method="cubic")

    # draw everything
    for ix, row in enumerate(bicubic):
        
        for jx, pixel in enumerate(row):
            #print(4)
#             startpoint=(displayPixelHeight * ix,displayPixelWidth * jx)
#             endpoint =(displayPixelHeight,displayPixelWidth)
#             colour = colors[constrain(int(pixel), 0, COLORDEPTH - 1)]
#             thickness = 0
            #print(ix)
            #print(round(displayPixelHeight))
            #print(displayPixelHeight * ix)
            #print(displayPixelWidth)
            #print(displayPixelWidth * jx)
            startpoint=(round(displayPixelHeight) * ix,round(displayPixelWidth) * jx)
            endpoint =(240,240)
            colour = colors[constrain(int(pixel), 0, COLORDEPTH - 1)]
            thickness = -1
            cv2.rectangle(img1,startpoint,endpoint,colour,thickness)
################################
            ###################
            
# read the pixels
    pixels = []

    for row in sensor2.pixels:
        pixels = pixels + row
    pixels = [map_value(p, MINTEMP, MAXTEMP, 0, COLORDEPTH - 1) for p in pixels]

    # perform interpolation
    bicubic = griddata(points, pixels, (grid_x, grid_y), method="cubic")

    # draw everything
    for ix, row in enumerate(bicubic):
        
        for jx, pixel in enumerate(row):
            #print(4)
#             startpoint=(displayPixelHeight * ix,displayPixelWidth * jx)
#             endpoint =(displayPixelHeight,displayPixelWidth)
#             colour = colors[constrain(int(pixel), 0, COLORDEPTH - 1)]
#             thickness = 0
            #print(ix)
            #print(round(displayPixelHeight))
            #print(displayPixelHeight * ix)
            #print(displayPixelWidth)
            #print(displayPixelWidth * jx)
            startpoint=(round(displayPixelHeight) * ix,round(displayPixelWidth) * jx)
            endpoint =(240,240)
            colour = colors[constrain(int(pixel), 0, COLORDEPTH - 1)]
            thickness = -1
            cv2.rectangle(img2,startpoint,endpoint,colour,thickness)


    cv2.imwrite(photopath+'/Image.jpg', img1)
    cv2.imwrite(photopath+'/Image2.jpg', img2)
    
    #presentDate = datetime.datetime.now()
    #Unix = datetime.datetime.timestamp(presentDate)*1000
    #cv2.imwrite(photopath+'/'+ Unix+'.jpg', img1)
    #cv2.imwrite(photopath+'/'+ Unix+'.jpg', img2)
# close all windows
cv2.destroyAllWindows()
