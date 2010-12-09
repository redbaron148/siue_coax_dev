#!/usr/bin/env python
import roslib; roslib.load_manifest('coax_client')
import rospy
from coax_client.msg import CoaxStateFiltered

data = "time,accel[0],accel[1],accel[2],global_accel[0],global_accel[1],global_accel[2]"

filename = "tmp.csv"

def callback(state):
    #rospy.loginfo(rospy.get_name()+"I heard something, adding to data")
    global data
    data += "\n"+`state.header.stamp.to_sec()`
    data += ","+`state.accel[0]`+","+`state.accel[1]`+","+`state.accel[2]`
    data += ","+`state.global_accel[0]`+","+`state.global_accel[1]`+","+`state.global_accel[2]`
    #data += ","+`filtered0`+","+`filtered1`+","+`filtered2`
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/coax_filtered/state", CoaxStateFiltered, callback)
    rospy.spin()
    global data
    newfile = open(filename,'w')
    newfile.write(data)
    newfile.close()

if __name__ == '__main__':
    listener()
