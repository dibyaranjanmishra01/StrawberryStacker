#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file
This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:
	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
         
    
'''
from cmath import pi
from logging import exception
import math
from multiprocessing.sharedctypes import Value
from pickletools import uint8
from turtle import distance, position
from gazebo_ros_link_attacher.srv import Gripper
from sensor_msgs.msg import Image
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np
import math
import cv2 as cv
import cv2.aruco as aruco
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import multiprocessing
from std_msgs.msg import UInt8


row_5 = 4*(1-1)+1
row_7 = 4*(7-1)+1
row_8 = 4*(2-1)+1
row_13 = 4*(13-1)+1
drone0 = {1:row_5,2:row_7}
drone1 = {1:row_8,2:row_13}

class offboard_control:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('multidrone', anonymous=True)

    def setArm(self, drone):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service(drone+'/mavros/cmd/arming', 30)
        try:
            armService = rospy.ServiceProxy(
                drone+'/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

        # Similarly delacre other service proxies

    def offboard_set_mode(self, drone):
        rospy.wait_for_service(drone+'/mavros/set_mode')
        try:
            modeService = rospy.ServiceProxy(
                drone+'/mavros/set_mode', mavros_msgs.srv.SetMode)
            modeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("Mode Setting call failed: %s" % e)
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure

    def land_set_mode(self,drone):
        rospy.wait_for_service(drone+'/mavros/set_mode')
        try:
            modeService = rospy.ServiceProxy(drone+
                '/mavros/set_mode', mavros_msgs.srv.SetMode)
            modeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print("Mode Setting call failed: %s" % e)

    def activate_gripper(self, drone, activate):
        rospy.wait_for_service(drone+'/activate_gripper')
        try:
            grippingService = rospy.ServiceProxy(
                drone+'/activate_gripper', Gripper)
            grippingService(activate)
        except rospy.ServiceException as e:
            print("Mode Setting call failed: %s" % e)


def getArucoPosition(img, Detected_ArUco_markers):
    for id in Detected_ArUco_markers:
        green_x = Detected_ArUco_markers[id][1][0]
        green_y = Detected_ArUco_markers[id][1][1]
        grey_x = Detected_ArUco_markers[id][0][0]
        grey_y = Detected_ArUco_markers[id][0][1]
        pink_x = Detected_ArUco_markers[id][2][0]
        pink_y = Detected_ArUco_markers[id][2][1]
        white_x = Detected_ArUco_markers[id][3][0]
        white_y = Detected_ArUco_markers[id][3][1]
        x_center = ((grey_x+pink_x)/2+(white_x+green_x)/2)/2
        y_center = ((grey_y+pink_y)/2+(white_y+green_y)/2)/2
    return x_center, y_center


class stateMoniter:
    def __init__(self):
        self.state = State()
        self.bridge = CvBridge()
        self.image_detected = False
        self.positionAcquired = False
        self.dis = 50
        self.id = -1

        # Instantiate a setpoints message
    def posCb(self, msg):
        self.curr_pose = msg
        x = self.curr_pose.pose.orientation.x
        y = self.curr_pose.pose.orientation.y
        z = self.curr_pose.pose.orientation.z
        w = self.curr_pose.pose.orientation.w
        self.pitch = math.asin(-2.0*(x*z - w*y)) #in radians

    def processData(self, data):
        print(data)
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            return img
        except exception:
            return None

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    def gripperCb(self, ready):
        self.gripper_ready = ready

    def image_callback(self, data):
        img = None
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except exception:
            pass
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)
        Detected_ArUco_markers = {}
        if ids != None and len(ids) > 0:
            #valid aruco can be processed for location
            #print("DETECTED!!")
            self.id = ids[0][0]
            for i in range(0, len(ids)):
                Detected_ArUco_markers[ids[i][0]] = corners[i][0]
            x_center = img.shape[1]/2
            y_center = img.shape[0]/2
            x, y = getArucoPosition(img, Detected_ArUco_markers)
            # if(math.sqrt((x_center-x)**2+(y_center-y)**2)<50):
            #     self.positionAcquired = True

            if(abs(x-x_center) < 25):
                if(abs(x-x_center) < self.dis):
                    self.dis = x-x_center
                    print("UPDATED")
                    self.positionAcquired = True
            self.image_detected = True

    # Create more callback functions for other subscribers


def printPose(target):
    print(target.pose.position.x, ",",
          target.pose.position.y, ",", target.pose.position.z)


def isComplete(curr_pose, target_pose):
    if(curr_pose.pose.position.x-target_pose.pose.position.x < 0.5 and curr_pose.pose.position.y-target_pose.pose.position.y < 0.5 and curr_pose.pose.position.z-target_pose.pose.position.z < 0.5):
        return True
    return False


def dist(curr_pose, target_pose):
    x = (curr_pose.pose.position.x-target_pose.pose.position.x)**2
    y = (curr_pose.pose.position.y-target_pose.pose.position.y)**2
    z = (curr_pose.pose.position.z-target_pose.pose.position.z)**2
    return math.sqrt(x+y+z)


def hasReached(curr_pose, target_pose, tolerance):
    if(abs(curr_pose.x-target_pose.x) < tolerance and abs(curr_pose.y-target_pose.y) < tolerance and abs(curr_pose.z-target_pose.z) < tolerance):
        return True
    return False

def printMsg(msg):
    print("ROW NO = ",msg)

def droneControl(drone,o_x,o_y,drone_dict):
    stateMt = stateMoniter()
    ofb_ctl = offboard_control()
    # Initialize subscriber
    rospy.Subscriber("/"+drone+"/mavros/state", State, stateMt.stateCb)
    rospy.Subscriber("/"+drone+"/mavros/local_position/pose",
                     PoseStamped, stateMt.posCb)
    rospy.Subscriber("/"+drone+"/camera/image_raw",
                     Image, stateMt.image_callback)
    rospy.Subscriber("/"+drone+"/gripper_check", String, stateMt.gripperCb)
    rospy.Subscriber("/spawn_info", UInt8,printMsg)
    local_pos_pub = rospy.Publisher(
        "/"+drone+"/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    # Specify the rate
    rate = rospy.Rate(20.0)
    while not stateMt.state.connected:
        rate.sleep()
    print("FCU connection established with"+drone)
    # Make the list of setpoints
    setpoints = []  # List to setpoint
    # Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 4
    row5 = PoseStamped()
    row5.pose.position.x = 0
    row5.pose.position.y = drone_dict[1]-o_y
    row5.pose.position.z = 4
    setpoints.append(row5)
    dummy = row5
    setpoints.append(dummy)

    setpoints_ = []
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 4
    row_ = PoseStamped()
    row_.pose.position.x = 0
    row_.pose.position.y = drone_dict[2]-o_y
    row_.pose.position.z = 4
    setpoints_.append(row_)
    dummy = row_
    setpoints_.append(dummy)

    # Arming the drone
    while not (stateMt.state.armed):
        ofb_ctl.setArm(drone)
        rate.sleep()
    print("Armed "+drone)
    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()
    while not stateMt.state.mode == "OFFBOARD":
        ofb_ctl.offboard_set_mode(drone)
        rate.sleep()
    print("OFFBOARD mode "+drone)
    
    blue = PoseStamped()
    blue.pose.position.x = 13.85+2*0.85-o_x
    blue.pose.position.y = -7.4-o_y
    blue.pose.position.z = 6
    blueLand = PoseStamped()
    blueLand.pose.position.x = 13.85+2*0.85-o_x
    blueLand.pose.position.y = -7.4-o_y
    blueLand.pose.position.z = 2.5
    blueAlt = PoseStamped()
    blueAlt.pose.position.x = 13.85+2*0.85-o_x
    blueAlt.pose.position.y = -7.4-o_y
    blueAlt.pose.position.z = 6

    red = PoseStamped()
    red.pose.position.x = 56.5+2*0.85-o_x
    red.pose.position.y = 64.75-o_y
    red.pose.position.z = 6
    redLand = PoseStamped()
    redLand.pose.position.x = 56.5+2*0.85-o_x
    redLand.pose.position.y = 64.75-o_y
    redLand.pose.position.z = 2.5
    redAlt = PoseStamped()
    redAlt.pose.position.x = 56.5+2*0.85-o_x
    redAlt.pose.position.y = 64.75-o_y
    redAlt.pose.position.z = 6

    target_pose = pos
    point_counter = 0
    hasPicked = False
    step = 3
    tolerance = 0.1 #0.1 works flawlessly
    
    i=1
    row = drone_dict[i]

    print(len(setpoints))

    while not rospy.is_shutdown():

        boxPose = PoseStamped()
        boxPose.pose.position.x = 0
        boxPose.pose.position.y = row-o_y
        boxPose.pose.position.z = -1

        if(hasReached(stateMt.curr_pose.pose.position, target_pose.pose.position, tolerance) and (not stateMt.positionAcquired or hasPicked)):
            print("Reached", target_pose.pose.position.x)
            print(tolerance)
            if(target_pose == pos):
                if(i==3):
                    ofb_ctl.land_set_mode(drone)
                    break
                step = 3
                tolerance = 0.1
                stateMt.dis = 50
                hasPicked = False
                stateMt.positionAcquired = False
                stateMt.image_detected = False   
            if(target_pose == blueLand or target_pose == redLand and i<3):
                ofb_ctl.activate_gripper(drone,False)
                step = 3
                stateMt.dis = 50
                stateMt.positionAcquired = False
                i = i+1
                if(i==1 or i==2):
                    row = drone_dict[i]
                else:
                    row = -1

            if(point_counter >= len(setpoints)) and not hasPicked:
                target_pose.pose.position.x = target_pose.pose.position.x+step
                target_pose.pose.position.y = row-o_y
                target_pose.pose.position.z = 4
            else:
                step = 3
                target_pose = setpoints[point_counter]
                print(target_pose.pose.position)
                point_counter = point_counter + 1

        elif stateMt.positionAcquired and stateMt.gripper_ready.data == "True" and not hasPicked:
            print(stateMt.gripper_ready)
            ofb_ctl.activate_gripper(drone, True)
            stateMt.image_detected = False
            tolerance = 0.5
            stateMt.positionAcquired = False
            hasPicked = True
            takeoff = PoseStamped()
            takeoff.pose.position.x = stateMt.curr_pose.pose.position.x
            takeoff.pose.position.y = row-o_y
            takeoff.pose.position.z = 4

            setpoints.insert(point_counter,takeoff)
            if(stateMt.id == 2):
                setpoints.insert(point_counter+1,blue)
                setpoints.insert(point_counter+2,blueLand)
                setpoints.insert(point_counter+3,blueAlt)
                setpoints.insert(point_counter+4,pos)
            else:
                setpoints.insert(point_counter+1,red)
                setpoints.insert(point_counter+2,redLand)
                setpoints.insert(point_counter+3,redAlt)
                setpoints.insert(point_counter+4,pos)
            if(i==1):
                setpoints.insert(point_counter+5,row_)
                # setpoints.insert(point_counter+6,dummy)
            #setpoints.insert(point_counter+5,row_)
            target_pose = setpoints[point_counter]
            point_counter = point_counter + 1
            
        if stateMt.image_detected:
            step = 1
        
        if stateMt.positionAcquired and not hasPicked:
            x = stateMt.curr_pose.pose.position.x
            boxPose.pose.position.x = x - stateMt.curr_pose.pose.position.z*math.tan(stateMt.pitch)
            target_pose = boxPose
            #printPose(target_pose)
        local_pos_pub.publish(target_pose)
        rate.sleep()

def main():
    p1 = multiprocessing.Process(target=droneControl,args=("edrone0",-1,1,drone0,))
    p2 = multiprocessing.Process(target=droneControl,args=("edrone1",-1,61,drone1,))
    # starting processes
    p1.start()
    p2.start()
    p1.join()
    p2.join()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

