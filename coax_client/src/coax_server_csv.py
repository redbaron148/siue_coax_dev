#!/usr/bin/env python
import roslib; roslib.load_manifest('coax_client')
import rospy
from coax_msgs.msg import CoaxState

data = "time,roll,pitch,yaw,gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2]"
data += ",zrange,zfiltered,pressure,hrange[0],hrange[1],hrange[2],rcChannel[0],"
data += "rcChannel[1],rcChannel[2],rcChannel[3],rcChannel[4],rcChannel[5],"
data += "rcChannel[6],rcChannel[7]"

#filtered0 = 0
#filtered1 = 0
#filtered2 = 0

filename = "tmp.csv"

def callback(state):
    #rospy.loginfo(rospy.get_name()+"I heard something, adding to data")
    #global filtered0
    #global filtered1
    #global filtered2
    #filtered0 = 0.5*filtered0+0.5*state.accel[0]
    #filtered1 = 0.6*filtered1+0.5*state.accel[0]
    #filtered2 = 0.8*filtered2+0.5*state.accel[0]
    global data
    data += "\n"+`state.header.stamp.to_sec()`+","+`state.roll`+","+`state.pitch`+","+`state.yaw`
    data += ","+`state.gyro[0]`+","+`state.gyro[1]`+","+`state.gyro[2]`
    data += ","+`state.accel[0]`+","+`state.accel[1]`+","+`state.accel[2]`
    data += ","+`state.zrange`+","+`state.zfiltered`+","+`state.pressure`
    data += ","+`state.hranges[0]`+","+`state.hranges[1]`+","+`state.hranges[2]`
    data += ","+`state.rcChannel[0]`+","+`state.rcChannel[1]`+","+`state.rcChannel[2]`
    data += ","+`state.rcChannel[3]`+","+`state.rcChannel[4]`+","+`state.rcChannel[5]`
    data += ","+`state.rcChannel[6]`+","+`state.rcChannel[7]`
    #data += ","+`filtered0`+","+`filtered1`+","+`filtered2`
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/coax_server/state", CoaxState, callback)
    rospy.spin()
    global data
    print "opened file data/tmp.csv\n"
    newfile = open(filename,'w')
    print "wrote data to file\n"
    newfile.write(data)
    print "closed file\n"
    newfile.close()

if __name__ == '__main__':
    listener()
