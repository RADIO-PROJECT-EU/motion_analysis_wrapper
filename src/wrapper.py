#!/usr/bin/env python
import os
import rospkg
import roslib, rospy
import subprocess, shlex
from datetime import datetime
from motion_analysis_msgs.msg import AnswerWithHeader
from radio_services.srv import InstructionAndStringWithAnswer

recorded_stand_up = False
started_walking = False
got_out_of_bed = False
first_time = True
object_sub = None
object_topic = ''
stood_up = False
human_sub = None
human_topic = ''
running = False
start_time = 0
logs_path = ''
robot_id = 0
finish1 = 0
finish2 = 0
finish3 = 0

def init():
    global human_topic, object_topic
    global robot_id
    global running, rospack
    rospy.init_node('motion_analysis_wrapper')
    human_topic = rospy.get_param("~human_topic", "/motion_analysis/event/human_transfer")
    object_topic = rospy.get_param("~object_topic", "/motion_analysis/event/object_tampered")
    robot_id = rospy.get_param("~robot_id", 0)
    if running:
        rospy.Subscriber(human_topic, AnswerWithHeader, humanCallback)
        rospy.Subscriber(object_topic, AnswerWithHeader, objectCallback)
    rospack = rospkg.RosPack()
    rospy.Service('/motion_analysis_wrapper/node_state_service', InstructionAndStringWithAnswer, nodeStateCallback)
    while not rospy.is_shutdown():
        rospy.spin()

def nodeStateCallback(req):
    global running, sub, logs_path, ros_visual_topic, start_time
    global object_topic, human_topic, human_sub, object_sub
    dt = datetime.now()
    start_time = dt.minute*60000000 + dt.second*1000000 + dt.microsecond
    if req.command == 0 and running:
        running = False
        try:
            human_sub.unregister()
            print 'Stopped motion analysis wrapper!'
        except:
            pass
        try:
            object_sub.unregister()
            print 'Stopped motion analysis wrapper!'
        except:
            pass
    elif req.command == 1 and not running:
        current_name = req.name
        current_repetition = req.repetition
        filename = 'official_log_bed_'+current_name+'_'+current_repetition+'_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S")+'.csv'
        logs_path = rospack.get_path('motion_analysis_wrapper') + '/logs/' + filename
        human_sub = rospy.Subscriber(human_topic, AnswerWithHeader, humanCallback)
        running = True
        with open(logs_path,'ab+') as f:
            f.write("Lying-Standing time, Standing-Walking time\n")
        print 'Started motion analysis wrapper (human mode)!'
    elif req.command == 2 and not running:
        current_name = req.name
        current_repetition = req.repetition
        filename = 'official_log_pill_'+current_name+'_'+current_repetition+'_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S")+'.csv'
        logs_path = rospack.get_path('motion_analysis_wrapper') + '/logs/' + filename
        object_sub = rospy.Subscriber(object_topic, AnswerWithHeader, objectCallback)
        running = True
        with open(logs_path,'ab+') as f:
            f.write("Pill intake date, Pill intake time\n")
        print 'Started motion analysis wrapper (object mode)!'
    return running

def humanCallback(msg):
    global got_out_of_bed, stood_up, started_walking, recorded_stand_up
    global logs_path, finish1, finish2, finish3, robot_id
    global start_time
    dt = datetime.now()
    if msg.event == 0 and not got_out_of_bed:
        got_out_of_bed =  True
        finish1 = dt.strftime("%H:%M:%S")
    elif msg.event == 1 and not stood_up:
        stood_up = True
        finish2 = dt.minute * 60000000 + dt.second * 1000000 + dt.microsecond
    elif msg.event == 2 and not started_walking and stood_up:
        started_walking = True
        finish3 = dt.minute * 60000000 + dt.second * 1000000 + dt.microsecond
    if stood_up:
        with open(logs_path,'ab+') as f:
            if not recorded_stand_up:
                f.write(str((finish2-start_time) / 1E6)+",")
                recorded_stand_up = True
            if started_walking:
                f.write(str((finish3-finish2) / 1E6)+"\n")
                human_sub.unregister()

def objectCallback(msg):
    global logs_path, object_sub
    dt = datetime.now()
    with open(logs_path,'ab+') as f:
        f.write(datetime.today().strftime("%d-%m-%Y")+','+dt.strftime("%H:%M:%S\n"))
    # Pill needs to run only once!
    object_sub.unregister()

if __name__ == '__main__':
    init()
