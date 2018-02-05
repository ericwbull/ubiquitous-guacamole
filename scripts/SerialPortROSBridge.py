#!/usr/bin/env python

import rospy
import serial
import struct
import re
import binascii
from enum import Enum

from std_msgs.msg import String
from beginner_tutorials.msg import Grideye
from beginner_tutorials.msg import GrideyeRaw
from beginner_tutorials.msg import NodeBytes

def NodeBytesReceivedFromROSFabric(data, bridge):
    dataAsBytes=bytearray(data.data)
    msg = "{}:{}".format(data.node, binascii.hexlify(dataAsBytes))
    print msg
    bridge.sendToSerialPort(msg)
    
def DataReceivedFromROSFabric(data, bridge):
   # Data received from the gateway application only
   # Message is formated as nodeid:data
   # We need to format the data as hex before sending to serial port
   # data is a bytearray
   # translate data to hex
    rospy.loginfo('{} command={}'.format(rospy.get_caller_id(),data.data))

    # separate the node number from the data
        # Parse node from message
    matchObj = re.match(r'(.*):(.*)', msg)
    node = 0
    data = ""
    if matchObj:
        node = int(matchObj.group(1))
        data = bytearray(matchObj.group(2))

    msg = "{}:{}".format(node,binascii.hexlify(data))
    bridge.sendToSerialPort(msg)

SERIAL_PORT='/dev/ttyS0'
BAUD_RATE=115200

class StreamID(Enum):
    RETURN_INFO = 1
    RETURN_GRIDEYE = 2
    RETURN_GRIDEYE_RAW = 3
    RETURN_DETECTION = 4
    RETURN_DETECTION_ARRAY = 5
    REQUEST_WIFI = 7
    REQUEST_IMAGE = 8
    REQUEST_DETECTION_ARRAY = 10
    RETURN_IMAGE = 9
    RETURN_PING = 66
    REQUEST_PING = 65
    UNKNOWN = 0

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
        self.pubNodeBytes = rospy.Publisher('byte_return_from_nodes', NodeBytes, queue_size=10)
        rospy.Subscriber('commands_to_nodes', String, DataReceivedFromROSFabric, self)
        rospy.Subscriber('byte_commands_to_nodes', NodeBytes, NodeBytesReceivedFromROSFabric, self)
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
        data = bytearray()
        other = ''
        if matchObj:
            node = int(matchObj.group(1))
            other = matchObj.group(2).strip()
        else:
            return node, data, other

        # Parse data from other
        matchObj = re.match(r'(.*)\[data\((\d+)\):([0-9a-f]*)\](.*)', other)
        
        if matchObj:
            other = matchObj.group(1) + matchObj.group(4)
            datalen = matchObj.group(2)
            data = matchObj.group(3)
        else:
            return node, data, other

        other = other.rstrip()
        print "node={} other={} data={}".format(node,other,data)

        # translate the data to a bytearray
        data = bytearray.fromhex(data)
        
        return node, data, other
        
    def sendToSerialPort(self,msg):
        self.ser.write(msg)
        self.ser.write('\n')

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            try: 
                (node, data, other) = self.readMessageFromSerialPort()

                streamid=StreamID.UNKNOWN
                try:
                    streamid=StreamID(data[0])
                except ValueError:
                    pass

                print "streamid={}".format(streamid)
                data = data[1:]
                
                if (streamid==StreamID.RETURN_INFO):
                    pubMsg = "[{}] {} {}\n".format(node,data,other)
                    print "publish '{}'".format(pubMsg)
                    self.pub.publish(pubMsg)
                elif (streamid==StreamID.RETURN_GRIDEYE):
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
                elif (streamid==StreamID.RETURN_GRIDEYE):
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
                elif (streamid==StreamID.RETURN_DETECTION):  # flircam detection
                    imageId,signalStrength,detection,safe = struct.unpack_from('>IIBB',data, 0)
                    signalHistogram = struct.unpack_from('>10I', data, 10);

                    receivedMsg = "[{0}] imageId={1} signalStrength=0x{2:04x} ({2}) detection={3} safe={4} signalHistogram={5} {6}".format(node,imageId,signalStrength,detection,safe,','.join(str(x) for x in signalHistogram), other)
                    print "{}".format(receivedMsg)

                    if detection:
                        pubMsg = "[{}] imageId:{} threat:{} threatSignal:{}:{} {}\n".format(node,imageId,imageId,signalStrength,','.join(str(x) for x in signalHistogram),other)
                        self.pub.publish(pubMsg)
                    if safe:
                        pubMsg = "[{}] imageId:{} safe:{} {}\n".format(node,imageId,imageId,other)
                        self.pub.publish(pubMsg)
                        
                elif (streamid==StreamID.RETURN_DETECTION_ARRAY):  # flircam detection&safebits
                    detection = struct.unpack_from('>9B',data, 0)
                    safe = struct.unpack_from('>9B',data, 9)

                    pubMsg="[{}] detectArray:".format(node)
                    for b in detection:
                        pubMsg += ":{:02x}".format(b)
                    pubMsg += " \n"
                    self.pub.publish(pubMsg)

                    pubMsg="[{}] safeArray:".format(node)
                    for b in safe:
                        pubMsg += ":{:02x}".format(b)
                    pubMsg += " \n"
                    self.pub.publish(pubMsg)
                elif (streamid==StreamID.RETURN_IMAGE):
                    # Forward to ROS
                    pubMsg = NodeBytes()
                    pubMsg.node = node
                    pubMsg.data = data
                    self.pubNodeBytes.publish(pubMsg)
                else:
                    print "unhandled streamid '{}'".format(streamid)

            except struct.error:
                pass
            rate.sleep()

if __name__ == '__main__':
    try:
        bridge = SerialPortROSBridge()
        bridge.run()
        
    except rospy.ROSInterruptException:
        pass

    except KeyboardInterrupt:
        pass
