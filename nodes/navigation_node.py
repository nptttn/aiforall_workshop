#!/usr/bin/env python3

import csv
import rospy
import rospkg
import actionlib
import tf2_ros
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from actionlib_msgs.msg import *
from your_package.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class NavigationLibrary(object):
    def __init__(self):
        rospy.init_node("navigation", anonymous=True)
        
        r = rospkg.RosPack()
        self.file_name = f"{r.get_path('your_package')}/csv/location.csv"

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("=== Wait for movebase action ===")
        self.move_base.wait_for_server()
        rospy.loginfo("=== Connected movebase action server ===")

        self.nav_to_loc_ser = rospy.Service("/nav/nav_to_location", NavToLocation, self.nav_to_loc_callback)
        self.save_location_ser = rospy.Service("/nav/save_location", SaveLocation, self.save_location_callback)
    
    def nav_to_loc_callback(self, data):
        location_name = data.location_name

        response = NavToLocationResponse()
        response.success = False

        self.go_to_location(location_name)

        success = self.move_base.wait_for_result(rospy.Duration(15))
        state = self.move_base.get_state()

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            response.success = True

        return response
    
    def save_location_callback(self, data):
        ret = self.save_position(data.location_name)
        if ret:
            return SaveLocationResponse(True)
        return SaveLocationResponse(False)
    
    def save_position(self, position_name):
        try:
            x,y,theta = self.get_position() # type: ignore
            thisdict = self.read_csv()
            thisdict[position_name] = [x,y,theta]
            thislist = []
            for location_name in thisdict:    
                thislist.append([location_name,thisdict.get(location_name)[0],thisdict.get(location_name)[1],thisdict.get(location_name)[2]])   #type: ignore
            
            with open(self.file_name, "w") as csv_file:
                csv_writer = csv.writer(csv_file,delimiter=',')
                for line in thislist:
                    csv_writer.writerow(line)
            return True

        except Exception as e:
            print(e)
            return False
        
    def get_position(self):
        self.listener = tf.TransformListener() 
        self.rate = rospy.Rate(1)
        get_position = False

        while not rospy.is_shutdown() and not get_position:
            try:
                trans = self.listener.lookupTransform("map", "base_footprint", rospy.Time())
                if trans != None:
                    get_position = True
                    print(trans)
                    rot = trans[1]
                    euler = euler_from_quaternion(rot)
                    return  trans[0][0], trans[0][1], euler[2] # type: ignore
        
            except Exception as e: # type: ignore
                rospy.logdebug(f"Error to get tf: {e}")
                self.rate.sleep()
                continue
    
    def go_to_location(self, location_name):
        x,y,theta = self.read_location(location_name)
        x,y,theta = float(x), float(y), float(theta)
        q = quaternion_from_euler(0,0,theta)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(q[0], q[1], q[2], q[3]))
        print(goal)
        self.move_base.send_goal(goal)

    def read_location(self, location_name):
        dict_position = self.read_csv()
        print(dict_position[location_name])
        return dict_position[location_name]
    
    def read_csv(self):
        thisdict = {}
        with open(self.file_name, "r") as csv_file:
            csv_reader = csv.reader(csv_file,delimiter=',')
            for row in csv_reader:
                thisdict[row[0]] = [row[1], row[2], row[3]]
        return thisdict


if __name__ == "__main__":
    nav = NavigationLibrary()
    rospy.spin()
