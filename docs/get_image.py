#!/usr/bin/env python3

#**********************************************************************
# Program:  get_image.py
# Description:
#    Interactive program to get image from TurtleBot4 and display 
#    and write to a file.
#----------------------------------------------------------------------
# Author: Andrew Kostiuk
# Copyright (c) 2025 University of Saskatchewan
#----------------------------------------------------------------------

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image

# System imports
from select import select
import sys
import termios
import tty
import math

import subprocess
import time
import threading
import datetime

# Other imports
import cv2
from cv_bridge import CvBridge

VERSION = "Get Image 2025-11-18.1"
IMAGE_TOPIC = "/image_rgb"
#IMAGE_TOPIC = "/oakd/rgb/preview/image_raw"
KEY_TIMEOUT = 0.1

class g:
    controller = None
    executor = None
    startTime = time.time_ns()
    imageMsg = None
    imageMsgAvailable = False
    image = None
    imageAvailable = False
    br = CvBridge()
    imageIndex = 0
    
#**********************************************************************
# MyPrint
#----------------------------------------------------------------------
def MyPrint(str):
    deltaTime = time.time_ns() - g.startTime
    secs = int(deltaTime / 1000000000.0)
    msecs = int((deltaTime - secs * 1000000000.0) / 1000000)
    print(f"{secs:03d}.{msecs:03d} {str}\r")
    sys.stdout.flush()

#**********************************************************************
# RobotControl
#----------------------------------------------------------------------
class RobotControl(Node):
    def __init__(self):
        try:
            super().__init__(f"RobotControl")
            self.image_subscriber = self.create_subscription(Image, IMAGE_TOPIC, self.process_camera, 10)
            MyPrint(f"RobotControl ready.")
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            MyPrint(f"RobotControl: EXCEPTION {e} Line {exc_tb.tb_lineno}")

    def process_camera(self, msg):
        try:
            #MyPrint(f"RobotControl got image.")
            g.imageMsg = msg
            g.imageMsgAvailable = True
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            MyPrint(f"RobotControl.process_camera: EXCEPTION {e} Line {exc_tb.tb_lineno}")

#**********************************************************************
# SaveImage
#----------------------------------------------------------------------
def SaveImage():
    try:
        MyPrint("SaveImage")
        if g.imageAvailable == True:
            g.imageIndex = g.imageIndex + 1
            filename = f"image{g.imageIndex}.png"
            success = cv2.imwrite(filename, g.image)
            if success:
                MyPrint(f"Image saved successfully as {filename}")
                g.imageAvailable = False
            else:
                MyPrint(f"Failed to save image as {filename}")            
        else:
            MyPrint("No image available")
    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        MyPrint(f"SaveImage: EXCEPTION {e} Line {exc_tb.tb_lineno}")
        
#**********************************************************************
# ViewImage
#----------------------------------------------------------------------
def ViewImage():
    try:
        MyPrint("ViewImage")
        if g.imageMsgAvailable == True:
            g.image = g.br.imgmsg_to_cv2(g.imageMsg, "bgr8")
            g.imageAvailable = True
            cv2.imshow("Window1", g.image)
            cv2.waitKey(200)
        else:
            MyPrint("No image available")
    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        MyPrint(f"ViewImage: EXCEPTION {e} Line {exc_tb.tb_lineno}")
        
#**********************************************************************
# Keyboard routines
#----------------------------------------------------------------------
def getKey(settings, timeout):
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    return key

def saveTerminalSettings():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    return settings

def restoreTerminalSettings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

#**********************************************************************
# Display Help
#----------------------------------------------------------------------
def DisplayHelp():
    MyPrint("=======================================================")
    MyPrint("Help:")
    MyPrint("    s       Save image")
    MyPrint("    v       View current image")
    MyPrint("    Ctrl-C  Exit program")
    MyPrint("-------------------------------------------------------")
    MyPrint(" View image first and then save to file")
    MyPrint("-------------------------------------------------------")
    
#**********************************************************************
# Keyboard thread
#----------------------------------------------------------------------
def keyboard():
    try:
        g.settings = saveTerminalSettings()
        DisplayHelp()
        
        while True:
            #MyPrint(f"{g.front_distance} {g.back_distance} {g.left_distance} {g.right_distance} {g.up_distance}")

            key = getKey(g.settings, KEY_TIMEOUT)
            
            # Check for empty key (i.e., timeout)
            if key != '':
                # Exit on Control-C
                if key == '\x03':
                    MyPrint("Exiting...")
                    break
                    
                match key:
                    case '?':
                        DisplayHelp()
                    case '\x0d':
                        MyPrint(f"{VERSION}, '?' for help")
                    case 's':
                        SaveImage()
                    case 'v':
                        ViewImage()
                    case _:
                        #hex_value = hex(ord(key))
                        #MyPrint(f"The hexadecimal value of '{key}' is: {hex_value}")
                        MyPrint('')

    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        MyPrint(f"keyboard: EXCEPTION:keyboard {e} Line {exc_tb.tb_lineno}")
    finally:
        #MyPrint("Finally")
        g.executor.shutdown()
        restoreTerminalSettings(g.settings)

#**********************************************************************
# Main
#----------------------------------------------------------------------
def main(args=None):
    try:
        MyPrint(f"{VERSION}")

        rclpy.init(args=args)

        t = threading.Thread(target=keyboard)
        t.start()

        g.controller = RobotControl()
        g.executor = SingleThreadedExecutor()
        g.executor.add_node(g.controller)
        g.executor.spin()
        
    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        MyPrint(f"main: EXCEPTION {e} Line {exc_tb.tb_lineno}")
        
if __name__ == "__main__":
    main()
