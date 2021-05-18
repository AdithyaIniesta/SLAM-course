#!/usr/bin/env python
import math 
import time
import rospy
import actionlib
from actionlib_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, PointStamped


class NavigateToGoals():

    def __init__(self):

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.count_goals = 0
        self.total_time = 0.0
        self.is_start = True
        self.max_no_of_markers = 100

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size = 100)
        self.mouse_click_sub = rospy.Subscriber('/clicked_point', PointStamped, self.mouse_click)
        self.amcl_sub = rospy.Subscriber ('/amcl_pose', PoseWithCovarianceStamped, self.get_amcl_pose) 
        
        rospy.on_shutdown(self.shutdown)
        points_seq = rospy.get_param('navigate_to_goals/p_seq')
        self.yaw_seq = rospy.get_param('navigate_to_goals/yea_seq')

        self.number_of_goals = len(self.yaw_seq)
        self.points = [points_seq[i:i + self.number_of_goals] for i in range(0, len(points_seq), self.number_of_goals)]

        rospy.Rate(1)
        rospy.sleep(2)

        rospy.loginfo("-----------Robot's initial pose-----------")
        rospy.loginfo("start x: {x:.2f} m, start y: {y:.2f} m, start yaw: {yaw:.2f} degrees"\
            .format(x = self.start_x, y = self.start_y, yaw = self.start_yaw ))
        
        self.visualize_goal_points()
        self.visit_goals()
        self.print_total_time()
    
    # get initial pose of the robot
    def get_amcl_pose(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation 
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] 
        yaw = math.degrees(euler_from_quaternion(orientation_list)[2])

        if(self.is_start):
            self.start_x = msg.pose.pose.position.x
            self.start_y = msg.pose.pose.position.y
            self.start_yaw = yaw
            self.is_start = False           

    # function to navigate to mouse click points
    def mouse_click(self, msg):

        print("x: ", msg.point.x, "y: ", msg.point.y)
        start = time.time()
        status = self.go_to_goal(msg.point.x, msg.point.y, 0)
        end = time.time()
        if(status == True):
            self.total_time += (end - start)
        self.print_total_time()

    # visit all the goals mentioned within map co-ordinate system 
    def visit_goals(self):

        for position, yaw_angle in zip(self.points, self.yaw_seq):
            start = time.time()
            status = self.go_to_goal(position[0], position[1], yaw_angle)
            end = time.time()
            if(status == True):
                self.total_time += (end - start)
            rospy.sleep(2)

    def set_orientation_quaternion(self, obj, yaw_angle):

        orientation_x, orientation_y, orientation_z, orientation_w = \
                                                quaternion_from_euler(0, 0, yaw_angle)

        obj.pose.orientation.x = orientation_x
        obj.pose.orientation.y = orientation_y
        obj.pose.orientation.z = orientation_z
        obj.pose.orientation.w = orientation_w
        return obj

    # visulise all the goal points: Position and Orientation using arrows
    def visualize_goal_points(self):

        for i in range(self.number_of_goals):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp    = rospy.get_rostime()
            marker.ns = "window"
            marker.id = i
            marker.type = marker.ARROW
            marker.action = marker.ADD

            point = self.points[i]
            marker.pose.position =  Point(point[0], point[1], point[2])
            
            marker = self.set_orientation_quaternion(marker,self.yaw_seq[i])
            marker.scale.x = 1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            self.marker_pub.publish(marker)

    # navigate to goal pose using ROS navigation stack
    def go_to_goal(self, x, y, yaw):

        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")
        
        orientation_x, orientation_y, orientation_z, orientation_w = \
                                            quaternion_from_euler(0, 0, yaw)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position =  Point(x, y, 0)
        goal.target_pose = self.set_orientation_quaternion(goal.target_pose,yaw)

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            self.count_goals += 1
            return True

        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False
    
    # print the time taken to visit goals
    def print_total_time(self, terminate = False):
        
        if(terminate):
            rospy.loginfo("Traversal to goal '{count}' terminated:".format(count = self.count_goals))
            rospy.loginfo("Number of goals visited := {count}".format(count = self.count_goals - 1))
            rospy.loginfo("Total time to reach goals and back to initial pose:= {time:.2f} seconds "\
                .format(time = self.total_time))
        else:
            rospy.loginfo("********************Time****************")
            rospy.loginfo("Total time to reach {count} goals:= {time:.2f} seconds "\
                .format(count = self.count_goals, time = self.total_time)) 
    
    # Go back to initial pose, when the node is terminated
    def shutdown(self):

        self.print_total_time(terminate = False)
        start = time.time()
        self.go_to_goal(self.start_x, self.start_y, self.start_yaw)
        end = time.time()
        self.total_time += (end - start)
        self.print_total_time(terminate = True)
        rospy.loginfo("Ctrl-C !. Going back to initial pose")

if __name__ == '__main__':
    try:
        rospy.init_node('navigate_to_goals', anonymous=False)
        navigator = NavigateToGoals()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Quit program")



