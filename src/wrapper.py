#!/usr/bin/env python
import os
import rospkg
import roslib, rospy
from time import sleep
import subprocess, shlex
from datetime import datetime
from motion_analysis.msg import AnswerWithHeader
from motion_detection_sensor_status_publisher.msg import SensorStatusMsg

human_topic = ''
object_topic = ''
motion_detection_topic = ''
start_time = 0
max_seconds = 180
got_out_of_bed = False
stood_up = False
started_walking = False
finish1 = 0
finish2 = 0
finish3 = 0
wrote_official_human_file = False
logs_path = ''
rosbag_proc = None
started_rosbag = False
record_rosbag = False
robot_id = 0
image_topic = ''
motion_sub = None

def init():
    global human_topic, object_topic, start_time, max_seconds, logs_path, record_rosbag
    global robot_id, image_topic
    dt = datetime.now()
    print "start_time = ", start_time
    rospy.init_node('motion_analysis_wrapper')
    human_topic = rospy.get_param("~human_topic", "/motion_analysis/event/human_transfer")
    object_topic = rospy.get_param("~object_topic", "/motion_analysis/event/object_tampered")
    motion_detection_topic = rospy.get_param("~motion_detection_topic", "/motion_detection_sensor_status_publisher/status")
    image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
    max_seconds = rospy.get_param("~max_seconds", 180)#3 minutes default
    record_rosbag = rospy.get_param("~record_rosbag", False)
    robot_id = rospy.get_param("~robot_id", 0)
    #before we start listening to the motion_analysis info, let's just wait for the camera image to stabilize)
    #sleep(6)
    start_time = dt.minute*60000000 + dt.second*1000000 + dt.microsecond
    rospy.Subscriber(human_topic, AnswerWithHeader, humanCallback)
    rospy.Subscriber(object_topic, AnswerWithHeader, objectCallback)
    motion_sub = rospy.Subscriber(motion_detection_topic, SensorStatusMsg, motionSensorCallback)
    rospack = rospkg.RosPack()
    logs_path = rospack.get_path('motion_analysis_wrapper')+'/logs/'
    while not rospy.is_shutdown():
        rospy.spin()

def motionSensorCallback(msg):
    global start_time, motion_sub
    dt = datetime.now()
    start_time = dt.minute*60000000 + dt.second*1000000 + dt.microsecond
    motion_sub.unregister()

def humanCallback(msg):
    global max_seconds, got_out_of_bed, stood_up, started_walking, wrote_official_human_file
    global logs_path, finish1, finish2, finish3, rosbag_proc, started_rosbag, record_rosbag, robot_id
    global start_time
    if record_rosbag:
        if not started_rosbag:
            print 'Starting rosbag record'
            command = "rosbag record -o "+rospkg.RosPack().get_path("motion_analysis_wrapper")+"/rosbags/human_robotID_"+str(robot_id)+" "+image_topic+" "+human_topic
            command = shlex.split(command)
            rosbag_proc = subprocess.Popen(command)
            started_rosbag = True
    #shutdown after max_seconds. To trigger this, we need to get at least one message from motion_analysis
    rospy.Timer(rospy.Duration(max_seconds), suicide)
    if msg.event == 0 and not got_out_of_bed:
        got_out_of_bed =  True
        dt = datetime.now()
        #finish1 = dt.minute*60000000 + dt.second*1000000 + dt.microsecond
        finish1 = dt.strftime("%H:%M:%S")
    elif msg.event == 1 and not stood_up:
        stood_up = True
        dt = datetime.now()
        finish2 = dt.minute*60000000 + dt.second*1000000 + dt.microsecond
    elif msg.event == 2 and not started_walking:
        started_walking = True
        dt = datetime.now()
        finish3 = dt.minute*60000000 + dt.second*1000000 + dt.microsecond
    
    if not wrote_official_human_file and started_walking:
        wrote_official_human_file = True
        dt = datetime.now()
        with open(logs_path+'official_log_bed_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S")+'.csv','w+') as f:
            #f.write('## Robot ID ##\n')
            #f.write(str(robot_id)+'\n')
            #if got_out_of_bed:
            #    f.write('## Lying-Sitting ##\n')
            #    f.write(str(float(finish1-start_time)/1000000)+' seconds\n')
            #if got_out_of_bed:
            #    f.write('## WARNING ##\n')
            #    f.write('Human was detected out of bed, but not stood up! Potential fall at '+finish1+'!!\n')                
            f.write("Lying-Standing time, Standing-Walking time\n")
            if stood_up:
                #f.write('## Lying-Standing ##\n')
                #f.write(str(float(finish2-start_time)/1000000)+' seconds\n')
                f.write(str(float(finish2-start_time) / 1E6)+",")
            else:
                f.write(" ,")
            if started_walking:
                #f.write('## Standing-Walking ##\n')
                #f.write(str(float(finish3-finish2)/1000000)+' seconds\n')
                f.write(str(float(finish3-finish2) / 1E6)+"\n")
            else:
                f.write(" \n")

    '''
    with open(logs_path+'temp_log_'+datetime.today().strftime("%d-%m-%Y")+'.log','a+') as f:
        ans = ''
        if msg.event == 0:
            ans = 'Sitting'
        elif msg.event == 1:
            ans = 'Standing'
        elif msg.event == 2:
            ans = 'Walking'
        f.write(str(datetime.now().strftime("[%d-%m-%Y %H:%M:%S] ")) + ans +'\n')
    '''

def objectCallback(msg):
    global rosbag_proc, started_rosbag, record_rosbag
    if record_rosbag:
        if not started_rosbag:
            print 'Starting rosbag record'
            command = "rosbag record -o "+rospkg.RosPack().get_path("motion_analysis_wrapper")+"/rosbags/object_robotID_"+str(robot_id)+" "+image_topic+" "+object_topic
            command = shlex.split(command)
            rosbag_proc = subprocess.Popen(command)
            started_rosbag = True
    dt = datetime.now()
    first_time = False
    if not os.path.isfile(logs_path+'official_log_pill_'+datetime.today().strftime("%d-%m-%Y")+'.csv'):
        first_time = True
    with open(logs_path+'official_log_pill_'+datetime.today().strftime("%d-%m-%Y")+'.csv','ab+') as f:
        if first_time:
            f.write("Pill intake date, Pill intake time\n")
        f.write(datetime.today().strftime("%d-%m-%Y")+','+dt.strftime("%H:%M:%S\n"))
        #f.write('## Robot ID ##\n')
        #f.write(str(robot_id)+'\n')
        #f.write('---\n')
        #f.write('Pill was taken on ' + datetime.today().strftime("%d-%m-%Y")+' '+dt.strftime("%H:%M:%S.\n"))
        #f.write('---\n')
    print msg.event
    suicide()

def suicide():
    global rosbag_proc, record_rosbag
    if record_rosbag:
        print 'Killing rosbag record'
        rosbag_proc.send_signal(subprocess.signal.SIGINT)
    print 'Killing motion_analysis'
    command = "rosnode kill motion_analysis"
    command = shlex.split(command)
    subprocess.Popen(command)
    print 'Killing myself'
    rospy.signal_shutdown("This is the end.")


if __name__ == '__main__':
    init()
