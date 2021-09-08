#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import String

# Callbacks definition

def send_goal(msg):
    def active_cb(extra):
        rospy.loginfo("Goal pose being processed")

    def feedback_cb(feedback):
        rospy.loginfo("Current location: "+str(feedback))

    def done_cb(status, result):
        if status == 3:
            rospy.loginfo("Goal reached")
        if status == 2 or status == 8:
            rospy.loginfo("Goal cancelled")
        if status == 4:
            rospy.loginfo("Goal aborted")

    def publish_to_goal(pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = pos_x
        goal.target_pose.pose.position.y = pos_y
        goal.target_pose.pose.position.z = pos_z
        goal.target_pose.pose.orientation.x = ori_x
        goal.target_pose.pose.orientation.y = ori_y
        goal.target_pose.pose.orientation.z = ori_z
        goal.target_pose.pose.orientation.w = ori_w
        # navclient.send_goal(goal, done_cb, active_cb, feedback_cb) 
        return goal

    if "WASHROOM" in msg.data:
        rospy.loginfo("Lets go to ***WASHROOM***")
        navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        navclient.wait_for_server()

        goal = publish_to_goal(51.909995667663, -6.4480388553087185, 0.0, 3.591340040306092e-10, 4.533459819264499e-10, 0.1148778243272563, -0.9933796280767168)
        navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
        finished = navclient.wait_for_result()
        
        if not finished:
            rospy.logerr("Action server not available!")
        else:
            rospy.loginfo ( navclient.get_result())
    elif "ELEVATOR" in msg.data:
        rospy.loginfo("Lets go to ***ELEVATOR***")
        navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        navclient.wait_for_server()

        goal = publish_to_goal(46.20998709501258, -11.36549256620539, 0.0, 4.541229785717897e-10, -3.581488383644668e-10, -0.993127937919775, -0.11703375121398038)
        navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
        finished = navclient.wait_for_result()
        
        if not finished:
            rospy.logerr("Action server not available!")
        else:
            rospy.loginfo ( navclient.get_result())
    elif "REGISTRAR OFFICE" in msg.data:
        rospy.loginfo("Lets go to ***REGISTRAR OFFICE***")
        navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        navclient.wait_for_server()
        
        goal = publish_to_goal(19.90287652537776, -12.420276831887996, 0.0, 4.392953730405917e-10, -3.7622039411822073e-10, -0.9970473870312463, -0.07678872322264604)

        navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
        finished = navclient.wait_for_result()
        
        if not finished:
            rospy.logerr("Action server not available!")
        else:
            rospy.loginfo ( navclient.get_result())
    elif "ORIENTATION ROOM" in msg.data:
        rospy.loginfo("Lets go to ***ORIENTATION ROOM***")
        navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        navclient.wait_for_server()

        goal = publish_to_goal(86.14242814044641, -30.9590344961912, 0.0, -5.520301899794944e-10, -1.709521987017617e-10, 0.4662429753002288, 0.8846567062896151)

        navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
        finished = navclient.wait_for_result()
        
        if not finished:
            rospy.logerr("Action server not available!")
        else:
            rospy.loginfo ( navclient.get_result())
    elif "SERVERY" in msg.data:
        rospy.loginfo("Lets go to ***SEERVERY***")
        navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        navclient.wait_for_server()

        goal = publish_to_goal(57.39227561477901, -34.77762492574952, 0.0, -2.4763640027941326e-12, 5.776695829020438e-10, 0.7102962928476837, -0.7039028174164653)

        navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
        finished = navclient.wait_for_result()
        
        if not finished:
            rospy.logerr("Action server not available!")
        else:
            rospy.loginfo ( navclient.get_result())
    elif "BEST LAB" in msg.data:
        rospy.loginfo("Lets go to ***BEST LAB***")
        navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        navclient.wait_for_server()

        goal = publish_to_goal(60.16131213618102, -82.48932754983305, 0.0, 4.1350008651315765e-10, -4.044208818596418e-10, -0.9999421129857706, -0.010759678320117564)

        navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
        finished = navclient.wait_for_result()
        
        if not finished:
            rospy.logerr("Action server not available!")
        else:
            rospy.loginfo ( navclient.get_result())
            

if __name__ == "__main__":
	rospy.init_node('send_goal')
	rospy.Subscriber("goal_nav", String, send_goal)
	rospy.spin()