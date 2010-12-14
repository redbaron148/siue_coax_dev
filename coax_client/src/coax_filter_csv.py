#!/usr/bin/env python
import roslib; roslib.load_manifest('coax_client')
import rospy
import os.path
import datetime
from coax_client.msg import CoaxStateFiltered

data = "time,accel[0],accel[1],accel[2],global_accel[0],global_accel[1],global_accel[2],global_accel_avg[0],global_accel_avg[1],global_accel_avg[2]"

last_write = 0
data_is_dirty = 0
count = 0

def callback(state):
    global data
    global data_is_dirty
    global last_write
    global count
    #rospy.loginfo("heard something")
    data += "\n"+`state.header.stamp.to_sec()`
    data += ","+`state.accel[0]`+","+`state.accel[1]`+","+`state.accel[2]`
    data += ","+`state.global_accel[0]`+","+`state.global_accel[1]`+","+`state.global_accel[2]`
    data += ","+`state.global_accel_avg[0]`+","+`state.global_accel_avg[1]`+","+`state.global_accel_avg[2]`
    #data += ","+`state.global_vel_avg[0]`+","+`state.global_vel_avg[1]`+","+`state.global_vel_avg[2]`
    if not data_is_dirty:
        data_is_dirty = 1
    last_write = rospy.Time.now().secs
    count += 1
    
def listener():
    rospy.init_node('fstate_data_dump')
    rospy.Subscriber('/coax_filtered/state', CoaxStateFiltered, callback)
    global data_is_dirty
    global count
    filename = rospy.get_param('/fstate_data_dump/filename','tmp')
    time_diff = rospy.get_param('/fstate_data_dump/time_diff',1)
    if os.path.isfile("/home/aaron/ros_pkgs/siue_coax_dev/coax_client/"+filename+".csv"):
        rospy.logwarn("The file %s.csv already exists.", filename)
    rospy.loginfo("writing to file %s every %d seconds after recieving a message.", filename+".csv",time_diff);
    
    while not rospy.is_shutdown():
        if data_is_dirty and ((rospy.Time.now().secs-last_write) > time_diff and time_diff != 0):
            data_is_dirty = 0
            if os.path.isfile(filename+".csv"):
                if rospy.get_param("/fstate_data_dump/overwrite","true"):
                    rospy.logwarn("The file %s.csv already exists, overwriting now.", filename)
                else:
                    rospy.logwarn("The file %s.csv already exists, creating new file called %s.csv",filename, datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S.")+filename)
                    filename = datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S.")+filename
            rospy.loginfo("writing data to file %s.csv",filename)
            global data
            newfile = open("/home/aaron/ros_pkgs/siue_coax_dev/coax_client/data/"+filename+".csv",'w')
            newfile.write(data)
            newfile.close()
            rospy.loginfo("wrote %d messages",count);
            count = 0
    if time_diff == 0 or data_is_dirty:
        filename = datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S.")+filename
        newfile = open("/home/aaron/ros_pkgs/siue_coax_dev/coax_client/data/"+filename+".csv",'w')
        newfile.write(data)
        newfile.close()
        rospy.loginfo("wrote %d messages",count);

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException: pass
