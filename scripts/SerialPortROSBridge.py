#!/usr/bin/env python

import rospy
import serial
import struct
import re

from std_msgs.msg import String
from beginner_tutorials.msg import Grideye
from beginner_tutorials.msg import GrideyeRaw

def DataReceivedFromROSFabric(data, bridge):
    rospy.loginfo('{} command={}'.format(rospy.get_caller_id(),data.data))
    bridge.sendToSerialPort(data.data)

SERIAL_PORT='/dev/ttyS0'
BAUD_RATE=115200
    
class SerialPortROSBridge:
    def __init__(self):
        serialIsOpen = False
        while not serialIsOpen:
            try:
                self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
                serialIsOpen = True
            except serial.SerialException:
                print "Serial port did not open"
                pass
        
        
        self.pub = rospy.Publisher('telemetry_from_nodes', String, queue_size=10)
        self.pubgrideye = rospy.Publisher('grideye_pixels', Grideye, queue_size=10)
        self.pubgrideyeraw = rospy.Publisher('grideye_pixels_raw', GrideyeRaw, queue_size=10)
        rospy.Subscriber('commands_to_nodes', String, DataReceivedFromROSFabric, self)
        rospy.init_node('SerialPortROSBridge', anonymous=True)

    def readLineFromSerialPort(self):
        msg = ""
        i = 0
        while not msg:
            print "polling serial {}".format(i)
            i = i + 1
            msg = self.ser.readline()
            return msg

    def readDataFromSerialPort(self):
        size = struct.unpack('B', self.ser.read())[0];
        print "size from serial data = {}".format(size)
        msg = bytearray(size)

        for i in range(0, size):
            msg[i] = self.ser.read();
        return msg

    def readMessageFromSerialPort(self):
        msg = self.readLineFromSerialPort();
        print "message={}".format(msg)

        # Parse node from message
        matchObj = re.match(r'\s*\[(\d+)\](.*)', msg)
        node = 0
        trailer = ''
        if matchObj:
            node = int(matchObj.group(1))
            trailer = matchObj.group(2).strip()

        
        print "node={} trailer='{}'".format(node,trailer)
        
        data = self.readDataFromSerialPort();
#        print "data='{}' len={}".format(data,len(data)) 

        return node, data, trailer
        
    def sendToSerialPort(self,msg):
        msg = self.ser.write(msg)

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            (node, data, trailer) = self.readMessageFromSerialPort()

            streamid=int(struct.unpack_from('B', data)[0])
            print "streamid={}".format(streamid)
            data = data[1:]
                
            if (streamid==1):
                pubMsg = "[{}] {} {}\n".format(node,data,trailer)
                print "publish '{}'".format(pubMsg)
                self.pub.publish(pubMsg)
            elif (streamid==2):
                datalist = list(data)
                databytecount = len(datalist)
                pixelCount = (databytecount - 1) / 4
                start = struct.unpack_from('B', data)[0]

                print "start={} databytecount={} pixelCount={} data={} datalen={}".format(start,databytecount,pixelCount,datalist,len(data)) 
                unpackstr="{}f".format(pixelCount)
                pixels = struct.unpack(unpackstr, data[1:])
                grideyeMsg = Grideye()
                grideyeMsg.start = start
                grideyeMsg.pixels = pixels
            
                print "grideye msg={}".format(grideyeMsg) 
                self.pubgrideye.publish(grideyeMsg)
            elif (streamid==3):
                datalist = list(data)
                databytecount = len(datalist)
                pixelCount = (databytecount - 1) / 2
                start = struct.unpack_from('B', data)[0]

                print "start={} databytecount={} pixelCount={} data={} datalen={}".format(start,databytecount,pixelCount,datalist,len(data)) 
                unpackstr="{}h".format(pixelCount)
                pixels = struct.unpack(unpackstr, data[1:])
                grideyeMsg = GrideyeRaw()
                grideyeMsg.start = start
                grideyeMsg.pixels = pixels
            
                print "grideyeraw msg={}".format(grideyeMsg) 
                self.pubgrideyeraw.publish(grideyeMsg)
            else:
                print "unhandled streamid '{}'".format(streamid)
                    
            rate.sleep()

if __name__ == '__main__':
    try:
        bridge = SerialPortROSBridge()
        bridge.run()
        
    except rospy.ROSInterruptException:
        pass
