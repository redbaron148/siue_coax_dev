#!/usr/bin/env python
import roslib; roslib.load_manifest('coax_client')
import rospy
import os.path
import datetime
from coax_client.msg import CoaxLocalization

data = "time,global_accel_avg[0],global_accel_avg[1],global_accel_avg[2],global_vel_avg[0],global_vel_avg[1],global_vel_avg[2],position[0],position[1],position[2]"

count = 0

def callback(state):
    global data
    global count
    data += "\n"+`state.header.stamp.to_sec()`
    data += ","+`state.global_accel_avg[0]`+","+`state.global_accel_avg[1]`+","+`state.global_accel_avg[2]`
    data += ","+`state.global_vel_avg[0]`+","+`state.global_vel_avg[1]`+","+`state.global_vel_avg[2]`
    data += ","+`state.position[0]`+","+`state.position[1]`+","+`state.position[2]`
    count += 1
    #print "heard something"
    
def listener():
    rospy.init_node('csv_dd')
    rospy.Subscriber('/coax_localization/state', CoaxLocalization, callback)
    global count
    global data
    filename = rospy.get_param('filename','tmp')
    path = rospy.get_param('path','')
    overwrite = rospy.get_param("overwrite","true")
    full_filename = path+filename+".csv"
    
    rospy.spin()
    
    if os.path.isfile(full_filename):
        if overwrite:
            rospy.logwarn("The file %s already exists, overwriting now.", full_filename)
        else:
            full_filename = path+datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S.")+filename+".csv"
            rospy.logwarn("The file already exists, creating new file %s",full_filename)
    rospy.loginfo("writing data to %s",full_filename)
    newfile = open(full_filename,'w')
    newfile.write(data)
    newfile.close()
    rospy.loginfo("wrote %d messages",count);
    count = 0
    data = '';

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException: pass
