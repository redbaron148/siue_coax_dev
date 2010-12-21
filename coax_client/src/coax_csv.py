#!/usr/bin/env python
import roslib; roslib.load_manifest('coax_client')
import rospy
import os.path
import datetime
from coax_client.msg import CoaxLocalization

data = "time,global_accel_avg[0],global_accel_avg[1],global_accel_avg[2],global_vel_avg[0],global_vel_avg[1],global_vel_avg[2],position[0],position[1],position[2]"

last_write = 0
data_is_dirty = 0
count = 0

def callback(state):
    global data
    global data_is_dirty
    global last_write
    global count
    data += "\n"+`state.header.stamp.to_sec()`
    #data += ","+`state.accel[0]`+","+`state.accel[1]`+","+`state.accel[2]`
    data += ","+`state.global_accel_avg[0]`+","+`state.global_accel_avg[1]`+","+`state.global_accel_avg[2]`
    data += ","+`state.global_vel_avg[0]`+","+`state.global_vel_avg[1]`+","+`state.global_vel_avg[2]`
    data += ","+`state.position[0]`+","+`state.position[1]`+","+`state.position[2]`
    if not data_is_dirty:
        data_is_dirty = 1
    last_write = rospy.Time.now().secs
    count += 1
    #print "heard something"
    
def listener():
    rospy.init_node('csv_dd')
    rospy.Subscriber('/coax_localization/state', CoaxLocalization, callback)
    global data_is_dirty
    global count
    global data
    filename = rospy.get_param('/csv_dd/filename','tmp')
    time_diff = rospy.get_param('/csv_dd/time_diff',1)
    path = rospy.get_param('/csv_dd/path','')
    overwrite = rospy.get_param("/csv_dd/overwrite","true")
    full_filename = path+filename+".csv"
    rospy.loginfo("writing to %s after %d seconds without recieving a message; overwrite: %s", full_filename,time_diff, overwrite)
    
    while not rospy.is_shutdown():
        if data_is_dirty and ((rospy.Time.now().secs-last_write) > time_diff and time_diff != 0):
            data_is_dirty = 0
            if os.path.isfile(full_filename):
                if overwrite:
                    rospy.logwarn("The file %s already exists, overwriting now.", full_filename)
                else:
                    rospy.logwarn("The file %s already exists, creating new file %s",full_filename, path+datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S.")+filename+".csv")
                    full_filename = path+datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S.")+filename+".csv"
            rospy.loginfo("writing data to %s",full_filename)
            newfile = open(full_filename,'w')
            newfile.write(data)
            newfile.close()
            rospy.loginfo("wrote %d messages",count);
            count = 0
            data = '';
    #print `data_is_dirty`+"\n\n\n"
    if data_is_dirty or (data_is_dirty and time_diff == 0):
        print "bwah!!!!!\n\n\n"
        if os.path.isfile(full_filename):
            if overwrite:
                rospy.logwarn("The file %s already exists, overwriting now.", full_filename)
            else:
                rospy.logwarn("The file %s already exists, creating new file %s",full_filename, path+datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S.")+filename+".csv")
                full_filename = path+datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S.")+filename+".csv"
        rospy.loginfo("writing data to %s",full_filename)
        newfile = open(full_filename,'w')
        newfile.write(data)
        newfile.close()
        rospy.loginfo("wrote %d messages",count);
        count = 0

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException: pass
