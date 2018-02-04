#!/usr/bin/env python

import rospy
from beginner_tutorials.msg import NodeBytes
import sys

def NodeBytesReceived(data,fetch):
    print "Recieved {}:{}".format(data.node, binascii.hexlify(data.data))
    # Write data to the file or save in memory until all the parts arrive
    
class ImageFetcher:
    def __init__(self, imageId):
        self.imageId = imageId
        self.pub = rospy.Publisher('byte_commands_to_nodes', NodeBytes, queue_size=10)
        rospy.Subscriber('byte_return_from_nodes', NodeBytes, NodeBytesReceived, self)
        self.blockReceived = []
        self.blockCountPending = 0
        rospy.init_node('FetchImage', anonymous=True)
        
    def saveCurrentToPGM(self):
        self.blockCountPending = 163
        self.blockReceived = [False for x in range(self.blockCountPending)]

        rate = rospy.Rate(10) # 10hz
#        while self.blockCountPending:
            # Keep requesting the missing parts until they all arrive
        pubMsg = NodeBytes()
        pubMsg.node = 5
        # streamid 8, imageid, imagetype (1=current), blocksize 59, range list (start, count), ...
        pubMsg.data = [8, self.imageId, 1, 59, 0, 163]
        print "publish"
        self.pub.publish(pubMsg)
        while not rospy.is_shutdown():
            rate.sleep()
        
if __name__ == '__main__':
    try:
        fetch = ImageFetcher(sys.argv[1])
        fetch.saveCurrentToPGM()
        
    except rospy.ROSInterruptException:
        pass

