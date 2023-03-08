# Library set up
##################################
import tkinter as tk
import time
import board
import adafruit_scd30
import busio
import RPi.GPIO as GPIO
import datetime
from mics6814 import MICS6814
import logging
import os
import math
import numpy as np
import pygame
import board
import csv
from scipy.interpolate import griddata


cwd = os.getcwd()
print(cwd)
###################################

# Csv setup

check = 0
print(check)
filename1 = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
print(filename1)
print('/')
filename = 'environment_reading_' + filename1+'.csv'
print(filename)

# current time and environment reading
with open(filename, 'w', newline='') as f:
    writer = csv.writer(f)
    header = [' Unix Time (secs) ','Temperature C ',' Humidity ',' Carbon Dioxide level (ppm) ', ' Oxidising gases (ppm) ', ' Reducing gases (ppm) ', ' NH3 (ppm) ' ,' Front ultrasonic distance(mm) ',' rear ultrasonic distance(mm) ']
    dw = csv.DictWriter(f, delimiter=',', fieldnames=header)
    dw.writeheader()


os.putenv("SDL_FBDEV", "/dev/fb1")

print('1')

# let the sensor initialize
time.sleep(0.1)


#################################################################

#gas sensor set up
##############################################################

##########################################################
def ultrasound(GPIO_TRIGGER, GPIO_ECHO, ref):

    #Libraries
    

    #GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    #set GPIO Pins
    #GPIO_TRIGGER = 17
    #GPIO_ECHO = 27


    #set GPIO direction (IN / OUT)
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)


    def distance():
        # set Trigger to HIGH
        GPIO.output(GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)

        StartTime = time.time()
        StopTime = time.time()

        # save StartTime
        while GPIO.input(GPIO_ECHO) == 0:
            StartTime = time.time()

        # save time of arrival
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()

        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2

        return distance

#     if __name__ == '__main__':
    try:
        #while True:
            dist = distance()
            #print ("Measured Distance" ,ref, " = %.1f cm" % dist)
            #time.sleep(1)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()

    return dist






def update():
    global check
    current_time = datetime.datetime.now().strftime(" %H:%M:%S")
    US1 =str(round(ultrasound(18,24,1)))
    US2 =str(round(ultrasound(17,27,2)))
    print("Ultrasound 1 is at " + US1 + "cm at " + current_time)
    print("Ultrasound 2 is at " + US2 + "cm at" + current_time)
    check+=1
    Temperature = -1
    CO2 = -1
    Humidity = -1
#     label.config(text= "Ultrasound 1 is at " + str(round(ultrasound(18,24,1))) + "cm at " + current_time)
#     label2.config(text = "Ultrasound 2 is at " + str(round(ultrasound(17,27,2))) + "cm at" + current_time)

    if scd.data_available:
        CO2 = str(round(scd.CO2))
        Temperature =str(round(scd.temperature))
        Humidity = str(round(scd.relative_humidity))
        print("CO2 is " + str(round(scd.CO2)) + " PPM at " + current_time)
        print("Temperature is" + str(round(scd.temperature)) + "degrees C at " + current_time)
        print("Humidity is " + str(round(scd.relative_humidity)) + "rH at" + current_time)
        
        check +=1
        success = 1
    return Temperature,CO2,Humidity,US1,US2
#         label3.config(text = "CO2 is" + str(round(scd.CO2)) + " PPM at " + current_time)
#         label4.config(text = "Temperature is" + str(round(scd.temperature)) + "degrees C at " + current_time)
#         label5.config(text = "Humidity is " + str(round(scd.relative_humidity)) + "rH at" + current_time)


#     root.after(500, update)




if __name__ == "__main__":
    i2c = busio.I2C(board.D3, board.D2,frequency=50000)
    scd = adafruit_scd30.SCD30(i2c)

#     

#!/usr/bin/env python3
    while True:
#         root.mainloop()
        
        gas = MICS6814()
        gas.set_led(10,10,10)
        gas.set_heater(76)

        logging.basicConfig(
            format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s',
            level=logging.INFO,
            datefmt='%Y-%m-%d %H:%M:%S')

        

        try:
            Temperature,CO2,Humidity,US1,US2 = update()
            readings = gas.read_all()
            ox = str(round(gas.read_oxidising()))
            red = str(round(gas.read_reducing()))
            nh3 = str(round(gas.read_nh3()))
            
            logging.info(" | ".join(str(readings).splitlines()))
            time.sleep(1.0)
            check+=1
        except KeyboardInterrupt:
            break
        
        #try:
        print(check)
        if check % 3 == 0:
            with open(filename, 'a', newline='') as f:
                writer = csv.writer(f)
                print(1)
                print(check)
                presentDate = datetime.datetime.now()
                Unix = datetime.datetime.timestamp(presentDate)*1000

            # temperature values are attained here
            # header = ['Temperature C ',' Humidity ',' Carbon Dioxide level (ppm) ', ' Oxidising gases (ppm) ', ' Reducing gases (ppm) ', ' NH3 (ppm) ' ,' Front ultrasonic distance(mm) ',' rear ultrasonic distance(mm) ']
                #print(2)
                data = [str(Unix),str(Temperature), str(Humidity), str(CO2),str(ox),str(red),str(nh3),str(US1),str(US2)]
                #print(3)
                writer.writerow(data)
                #print(4)
                
        check = 0       
        #except:
        #        print('Something went wrong with the data saving')

