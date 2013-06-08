#!/usr/bin/python

##################################################
#                                                #
#   Calder Phillips-Grafflin - WPI/ARC Lab       #
#                                                #
# Simple class interface to the onboard services #
# for inverse and forward kinematics on the PR2. #
#                                                #
##################################################

import roslib; roslib.load_manifest('simple_robot_kinematics')
import rospy
import math
import random
import time

from std_msgs.msg import String
from geometry_msgs.msg import *
import kinematics_msgs
from kinematics_msgs.msg import *
from kinematics_msgs.srv import *
import arm_navigation_msgs
from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *

class SimpleLeftArmKinematics:

    def __init__(self):
        #Set the planning scene
        rospy.wait_for_service("/environment_server/set_planning_scene_diff")
        self.planner_host = rospy.ServiceProxy("/environment_server/set_planning_scene_diff", SetPlanningSceneDiff)
        self.dummy_setup_req = SetPlanningSceneDiff._request_class()
        self.dummy_setup_res = self.planner_host.call(self.dummy_setup_req)
        #Set the IK solver info
        rospy.wait_for_service("pr2_left_arm_kinematics/get_ik_solver_info")
        self.ik_solver_host = rospy.ServiceProxy("pr2_left_arm_kinematics/get_ik_solver_info", GetKinematicSolverInfo)
        self.ik_solver_req = GetKinematicSolverInfo._request_class()
        self.ik_solver_res = self.ik_solver_host.call(self.ik_solver_req)
        #Set the FK solver info
        rospy.wait_for_service("pr2_left_arm_kinematics/get_fk_solver_info")
        self.fk_solver_host = rospy.ServiceProxy("pr2_left_arm_kinematics/get_fk_solver_info", GetKinematicSolverInfo)
        self.fk_solver_req = GetKinematicSolverInfo._request_class()
        self.fk_solver_res = self.fk_solver_host.call(self.fk_solver_req)
        #Set up the left arm forward kinematics service call
        rospy.wait_for_service("pr2_left_arm_kinematics/get_fk")
        self.left_FK_host = rospy.ServiceProxy("pr2_left_arm_kinematics/get_fk", GetPositionFK)
        #Set up the left arm collision-free inverse kinematics service call
        rospy.wait_for_service("pr2_left_arm_kinematics/get_constraint_aware_ik")
        self.left_CAIK_host = rospy.ServiceProxy("pr2_left_arm_kinematics/get_constraint_aware_ik", GetConstraintAwarePositionIK)
        #Set up the left arm inverse kinematics service call
        rospy.wait_for_service("pr2_left_arm_kinematics/get_ik")
        self.left_IK_host = rospy.ServiceProxy("pr2_left_arm_kinematics/get_ik", GetPositionIK)
        rospy.loginfo("PR2 Left arm kinematics services loaded")

    def RunConstraintAwareIK(self, desired_wrist_pose_stamped, seed_state=None):
        ik_request = GetConstraintAwarePositionIK._request_class()
        #Populate the header
        ik_request.timeout = rospy.Duration(5.0)
        ik_request.ik_request.ik_link_name = "l_wrist_roll_link"
        #Populate the desired pose
        ik_request.ik_request.pose_stamped = desired_wrist_pose_stamped
        #Populate the IK seed state
        ik_request.ik_request.ik_seed_state.joint_state.name = self.ik_solver_res.kinematic_solver_info.joint_names
        ik_request.ik_request.ik_seed_state.joint_state.position = []
        if (seed_state == None):
            for i in range(len(self.ik_solver_res.kinematic_solver_info.joint_names)):
                middle_position = (self.ik_solver_res.kinematic_solver_info.limits[i].min_position + self.ik_solver_res.kinematic_solver_info.limits[i].max_position) / 2.0
                ik_request.ik_request.ik_seed_state.joint_state.position.append(middle_position)
        else:
            ik_request.ik_request.ik_seed_state.joint_state.position = seed_state
        #Actually call the IK system
        ik_response = self.left_CAIK_host.call(ik_request)
        if (ik_response.error_code.val == ik_response.error_code.SUCCESS):
            return ik_response.solution.joint_state.position
        else:
            #Unable to find an IK solution
            return None

    def RunIK(self, desired_wrist_pose_stamped, seed_state=None):
        ik_request = GetPositionIK._request_class()
        #Populate the header
        ik_request.timeout = rospy.Duration(5.0)
        ik_request.ik_request.ik_link_name = "l_wrist_roll_link"
        #Populate the desired pose
        ik_request.ik_request.pose_stamped = desired_wrist_pose_stamped
        #Populate the IK seed state
        ik_request.ik_request.ik_seed_state.joint_state.name = self.ik_solver_res.kinematic_solver_info.joint_names
        ik_request.ik_request.ik_seed_state.joint_state.position = []
        if (seed_state == None):
            for i in range(len(self.ik_solver_res.kinematic_solver_info.joint_names)):
                middle_position = (self.ik_solver_res.kinematic_solver_info.limits[i].min_position + self.ik_solver_res.kinematic_solver_info.limits[i].max_position) / 2.0
                ik_request.ik_request.ik_seed_state.joint_state.position.append(middle_position)
        else:
            ik_request.ik_request.ik_seed_state.joint_state.position = seed_state
        #Actually call the IK system
        ik_response = self.left_IK_host.call(ik_request)
        if (ik_response.error_code.val == ik_response.error_code.SUCCESS):
            return ik_response.solution.joint_state.position
        else:
            #Unable to find an IK solution
            return None

    def RunFK(self, joint_state, target_frame_id, target_link_id):
        fk_request = GetPositionFK._request_class()
        fk_request.header.frame_id = target_frame_id
        if (type(target_link_id) == type([])):
            fk_request.fk_link_names = target_link_id
        else:
            fk_request.fk_link_names = [target_link_id]
        fk_request.robot_state.joint_state.name = self.fk_solver_res.kinematic_solver_info.joint_names
        fk_request.robot_state.joint_state.position = joint_state
        fk_response = self.left_FK_host.call(fk_request)
        if (fk_response.error_code.val == fk_response.error_code.SUCCESS):
            if (len(fk_response.pose_stamped) == 1):
                return fk_response.pose_stamped[0]
            else:
                return fk_response.pose_stamped
        else:
            return None

class SimpleRightArmKinematics:

    def __init__(self):
        #Set the planning scene
        rospy.wait_for_service("/environment_server/set_planning_scene_diff")
        self.planner_host = rospy.ServiceProxy("/environment_server/set_planning_scene_diff", SetPlanningSceneDiff)
        self.dummy_setup_req = SetPlanningSceneDiff._request_class()
        self.dummy_setup_res = self.planner_host.call(self.dummy_setup_req)
        #Set the IK solver info
        rospy.wait_for_service("pr2_right_arm_kinematics/get_ik_solver_info")
        self.ik_solver_host = rospy.ServiceProxy("pr2_right_arm_kinematics/get_ik_solver_info", GetKinematicSolverInfo)
        self.ik_solver_req = GetKinematicSolverInfo._request_class()
        self.ik_solver_res = self.ik_solver_host.call(self.ik_solver_req)
        #Set the FK solver info
        rospy.wait_for_service("pr2_right_arm_kinematics/get_fk_solver_info")
        self.fk_solver_host = rospy.ServiceProxy("pr2_right_arm_kinematics/get_fk_solver_info", GetKinematicSolverInfo)
        self.fk_solver_req = GetKinematicSolverInfo._request_class()
        self.fk_solver_res = self.fk_solver_host.call(self.fk_solver_req)
        #Set up the right arm forward kinematics service call
        rospy.wait_for_service("pr2_right_arm_kinematics/get_fk")
        self.right_FK_host = rospy.ServiceProxy("pr2_right_arm_kinematics/get_fk", GetPositionFK)
        #Set up the right arm collision-free inverse kinematics service call
        rospy.wait_for_service("pr2_right_arm_kinematics/get_constraint_aware_ik")
        self.right_CAIK_host = rospy.ServiceProxy("pr2_right_arm_kinematics/get_constraint_aware_ik", GetConstraintAwarePositionIK)
        #Set up the right arm inverse kinematics service call
        rospy.wait_for_service("pr2_right_arm_kinematics/get_ik")
        self.right_IK_host = rospy.ServiceProxy("pr2_right_arm_kinematics/get_ik", GetPositionIK)
        rospy.loginfo("PR2 Right arm kinematics services loaded")

    def RunConstraintAwareIK(self, desired_wrist_pose_stamped, seed_state=None):
        ik_request = GetConstraintAwarePositionIK._request_class()
        #Populate the header
        ik_request.timeout = rospy.Duration(5.0)
        ik_request.ik_request.ik_link_name = "r_wrist_roll_link"
        #Populate the desired pose
        ik_request.ik_request.pose_stamped = desired_wrist_pose_stamped
        #Populate the IK seed state
        ik_request.ik_request.ik_seed_state.joint_state.name = self.ik_solver_res.kinematic_solver_info.joint_names
        if (seed_state == None):
            for i in range(len(self.ik_solver_res.kinematic_solver_info.joint_names)):
                middle_position = (self.ik_solver_res.kinematic_solver_info.limits[i].min_position + self.ik_solver_res.kinematic_solver_info.limits[i].max_position) / 2.0
                ik_request.ik_request.ik_seed_state.joint_state.position.append(middle_position)
        else:
            ik_request.ik_request.ik_seed_state.joint_state.position = seed_state
        #Actually call the IK system
        ik_response = self.right_CAIK_host.call(ik_request)
        if (ik_response.error_code.val == ik_response.error_code.SUCCESS):
            return ik_response.solution.joint_state.position
        else:
            #Unable to find an IK solution
            return None

    def RunIK(self, desired_wrist_pose_stamped, seed_state=None):
        ik_request = GetPositionIK._request_class()
        #Populate the header
        ik_request.timeout = rospy.Duration(5.0)
        ik_request.ik_request.ik_link_name = "r_wrist_roll_link"
        #Populate the desired pose
        ik_request.ik_request.pose_stamped = desired_wrist_pose_stamped
        #Populate the IK seed state
        ik_request.ik_request.ik_seed_state.joint_state.name = self.ik_solver_res.kinematic_solver_info.joint_names
        if (seed_state == None):
            for i in range(len(self.ik_solver_res.kinematic_solver_info.joint_names)):
                middle_position = (self.ik_solver_res.kinematic_solver_info.limits[i].min_position + self.ik_solver_res.kinematic_solver_info.limits[i].max_position) / 2.0
                ik_request.ik_request.ik_seed_state.joint_state.position.append(middle_position)
        else:
            ik_request.ik_request.ik_seed_state.joint_state.position = seed_state
        #Actually call the IK system
        ik_response = self.right_IK_host.call(ik_request)
        if (ik_response.error_code.val == ik_response.error_code.SUCCESS):
            return ik_response.solution.joint_state.position
        else:
            #Unable to find an IK solution
            return None

    def RunFK(self, joint_state, target_frame_id, target_link_id):
        fk_request = GetPositionFK._request_class()
        fk_request.header.frame_id = target_frame_id
        if (type(target_link_id) == type([])):
            fk_request.fk_link_names = target_link_id
        else:
            fk_request.fk_link_names = [target_link_id]
        fk_request.robot_state.joint_state.name = self.fk_solver_res.kinematic_solver_info.joint_names
        fk_request.robot_state.joint_state.position = joint_state
        fk_response = self.right_FK_host.call(fk_request)
        if (fk_response.error_code.val == fk_response.error_code.SUCCESS):
            if (len(fk_response.pose_stamped) == 1):
                return fk_response.pose_stamped[0]
            else:
                return fk_response.pose_stamped
        else:
            return None
