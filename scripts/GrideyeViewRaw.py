#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from beginner_tutorials.msg import GrideyeRaw

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import time


def DataReceivedFromROSFabric(data, bridge):
#    print "received '{}'".format(data)
    
    for i in range(0,len(data.pixels)):
        bridge.pixels[data.start + i] = data.pixels[i]

    print " {}".format(len(data.pixels)),
        
    if (len(data.pixels) == 12):
        print
        print bridge.pixels.reshape(8,8)
        bridge.receiveDataCounter += 1
        print "receive count={}".format(bridge.receiveDataCounter)
        print




#    bridge.sendToSerialStream(data.data)

# Transfers messages, bidirectionally, between the SerialStream.Server (the gateway.js web app is on the other side) and ROS (serial port hardware is on the other side)
class GrideyeView:
    def __init__(self):
        np.set_printoptions(linewidth=130)
        rospy.Subscriber('grideye_pixels_raw', GrideyeRaw, DataReceivedFromROSFabric, self)
        rospy.init_node('GrideyeView', anonymous=True)
        self.pixels=np.arange(64)
        self.receiveDataCounter=1
        self.displayDataCounter=0
        plt.ion()
        
    def run(self):
        while True:
#        rospy.spin()
            dataCounterDiff = self.receiveDataCounter - self.displayDataCounter
            if (dataCounterDiff != 0):
                # if more data comes in while we are inside this block, then it will be
                # displayed in the next iteration.
                plt.figure(1)
                pixelGrid=self.pixels.reshape(8,8)
                plt.imshow(pixelGrid, interpolation='none', vmin=0, vmax=120)
                plt.figure(2)
                plt.imshow(pixelGrid, interpolation='none')
                self.displayDataCounter += dataCounterDiff
                print "display count={}".format(self.displayDataCounter)
                
            plt.pause(0.05)
            time.sleep(1)
            
if __name__ == '__main__':
    bridge = GrideyeView()
    bridge.run()
