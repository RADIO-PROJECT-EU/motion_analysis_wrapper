#!/usr/bin/env python
import roslib, rospy
from motion_analysis.msg import AnswerWithHeader
from motion_detection_sensor_status_publisher.msg import SensorStatusMsg
from datetime import datetime
import rospkg
import subprocess, shlex

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

def init():
    global human_topic, object_topic, start_time, max_seconds, logs_path
    dt = datetime.now()
    start_time = dt.minute*60000000 + dt.second*1000000 + dt.microsecond
    print "start_time = ", start_time
    rospy.init_node('motion_analysis_wrapper')
    human_topic = rospy.get_param("~human_topic", "/motion_analysis/event/human_transfer")
    object_topic = rospy.get_param("~object_topic", "/motion_analysis/event/object_tampered")
    motion_detection_topic = rospy.get_param("~motion_detection_topic", "/motion_detection_sensor_status_publisher/status")
    max_seconds = rospy.get_param("~max_seconds", 180)#3 minutes default
    rospy.Subscriber(human_topic, AnswerWithHeader, humanCallback)
    rospy.Subscriber(object_topic, AnswerWithHeader, objectCallback)
    rospy.Subscriber(motion_detection_topic, SensorStatusMsg, motionSensorCallback)
    rospack = rospkg.RosPack()
    logs_path = rospack.get_path('motion_analysis_wrapper')+'/logs/'
    '''
    dt = datetime.now()
    finish = dt.minute*60000000 + dt.second*1000000 + dt.microsecond
    print finito-start_time
    print float(finish-start_time)/1000000 #in seconds
    '''
    while not rospy.is_shutdown():  
        rospy.spin()

def motionSensorCallback(msg):
    global got_out_of_bed, start_time
    if not got_out_of_bed:
        dt = datetime.now()
        start_time = dt.minute*60000000 + dt.second*1000000 + dt.microsecond

def humanCallback(msg):
    global max_seconds, got_out_of_bed, stood_up, started_walking, wrote_official_human_file, logs_path, finish1, finish2, finish3
    #shutdown after max_seconds. To trigger this, we need to get at least one message from motion_analysis
    rospy.Timer(rospy.Duration(20), suicide)
    if msg.event == 0 and not got_out_of_bed:
        got_out_of_bed =  True
        dt = datetime.now()
        finish1 = dt.minute*60000000 + dt.second*1000000 + dt.microsecond
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
        with open(logs_path+'official_log_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S"),'w+') as f:
            if got_out_of_bed:
                f.write('##Start-Sitting##\n')
                f.write(str(float(finish1-start_time)/1000000)+' seconds\n')
            if stood_up:
                f.write('##Sitting-Standing##\n')
                f.write(str(float(finish2-start_time)/1000000)+' seconds\n')
            f.write('##Standing-Walking##\n')
            f.write(str(float(finish3-start_time)/1000000)+' seconds\n')

    with open(logs_path+'temp_log_'+datetime.today().strftime("%d-%m-%Y"),'a+') as f:
        ans = ''
        if msg.event == 0:
            ans = 'Sitting'
        elif msg.event == 1:
            ans = 'Standing'
        elif msg.event == 2:
            ans = 'Walking'
        f.write(str(datetime.now().strftime("[%d-%m-%Y %H:%M:%S] ")) + ans +'\n')

def objectCallback(msg):
    print msg.event

def suicide(arg):
    print 'Killing motion_analysis'
    command = "rosnode kill motion_analysis"
    command = shlex.split(command)
    subprocess.Popen(command)
    print 'Killing my self'
    rospy.signal_shutdown("This is the end.")


if __name__ == '__main__':
    init()