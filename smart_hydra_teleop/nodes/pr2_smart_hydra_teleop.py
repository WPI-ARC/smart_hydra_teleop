#!/usr/bin/python

import roslib; roslib.load_manifest('smart_hydra_teleop')
import rospy
import math
from copy import deepcopy

from visualization_msgs.msg import *
from razer_hydra.msg import *
from geometry_msgs.msg import *
import actionlib
import pr2_simple_kinematics
from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import *

class SmartHydraTeleop:

    def __init__(self, head_pointing_frame, execution_scaling_time, safe):
        self.head_pointing_frame = head_pointing_frame
        self.execution_scaling_time = execution_scaling_time
        self.safe = safe
        self.BUSY = False
        #Set the mesh model for marker display
        self.grippermodel ="package://pr2_description/meshes/gripper_v0/gripper_palm.dae"
        #Paddle definitions
        self.left = 0
        self.right = 1
        #Button definitions
        self.x_axis = 1 #number of up-down axis on analog stick
        self.y_axis = 0 #number of left-right axis on analog stick
        self.left_close = 4 #numbered button on left paddle
        self.left_open = 3 #numbered button on left paddle
        self.right_close = 3 #numbered button on right paddle
        self.right_open = 4 #numbered button on right paddle
        self.deadman_button = 5 #number of the left & right unmarked buttons
        self.gripper_step = 0.0001 #step size for gripper controller
        self.bumper = 0 #number of bumper button
        self.stick = 6 #number of the stick button
        #Storage for head pan/tilt state
        self.head_pan = 0.0
        self.head_tilt = 0.0
        #Storage for the gripper setpoints
        self.left_gripper = 0.08
        self.right_gripper = 0.08
        #Storage for the gripper states
        self.left_gripper_state = None
        self.right_gripper_state = None
        #Storage for exec control buttons
        self.leftexec = 0
        self.rightexec = 0
        #Storage for arm modes
        self.left_mode = -1
        self.right_mode = -1
        #Storage for arm states - valid values are U 'unknown', S 'succeeded', F 'failed'
        self.left_state = 'U'
        self.right_state = 'U'
        #Storage for the arm targets
        self.left_target = None
        self.right_target = None
        #Storage for hydra states
        self.left_hydra_state = None
        self.right_hydra_state = None
        #Storage for the arm joint states
        self.current_l_arm_joints = None
        self.current_r_arm_joints = None
        #Storage for the pose data returned from the arm cartesian controllers
        self.current_l_arm_pose = None
        self.current_r_arm_pose = None
        #Set up the publishers
        self.base_pub = rospy.Publisher('base_controller/command', Twist)
        self.marker_pub = rospy.Publisher("smart_hydra_teleop/markers", MarkerArray)
        #Set up the arm IK clients
        rospy.loginfo("Starting left & right arm IK clients...")
        self.left_arm_kinematics = pr2_simple_kinematics.SimpleLeftArmKinematics()
        self.right_arm_kinematics = pr2_simple_kinematics.SimpleRightArmKinematics()
        rospy.loginfo("Kinematics clients loaded")
        self.left_arm_client = actionlib.SimpleActionClient("l_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.left_arm_client.wait_for_server()
        self.right_arm_client = actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.right_arm_client.wait_for_server()
        rospy.loginfo("Arm clients loaded")
        self.left_gripper_client = actionlib.SimpleActionClient("l_gripper_controller/gripper_action", Pr2GripperCommandAction)
        self.left_gripper_client.wait_for_server()
        self.right_gripper_client = actionlib.SimpleActionClient("r_gripper_controller/gripper_action", Pr2GripperCommandAction)
        self.right_gripper_client.wait_for_server()
        rospy.loginfo("Gripper clients loaded")
        #Set up actionlib client for the head
        self.head_client = actionlib.SimpleActionClient('head_traj_controller/point_head_action', PointHeadAction)
        self.head_client.wait_for_server()
        rospy.loginfo("Head clients loaded")
        #Subscribe to the arm controllers for joint and pose data
        rospy.Subscriber("l_arm_controller/state", JointTrajectoryControllerState, self.l_arm_joint_cb)
        rospy.Subscriber("r_arm_controller/state", JointTrajectoryControllerState, self.r_arm_joint_cb)
        rospy.Subscriber("l_arm_cart_controller/state/pose", PoseStamped, self.l_arm_pose_cb)
        rospy.Subscriber("r_arm_cart_controller/state/pose", PoseStamped, self.r_arm_pose_cb)
        #Subscribe to the gripper controllers for gripper state data
        rospy.Subscriber("l_gripper_controller/state", JointControllerState, self.left_gripper_state_cb)
        rospy.Subscriber("r_gripper_controller/state", JointControllerState, self.right_gripper_state_cb)
        rospy.loginfo("Waiting for data from the controllers...")
        while (self.current_l_arm_joints == None or self.current_r_arm_joints == None):
            rospy.sleep(0.01)
        #Move the arms to the default start pose
        rospy.loginfo("Setting the arms to a known starting state")
        [self.left_target, self.right_target] = self.set_arms()
        #Subscribe to the Hydra
        rospy.Subscriber("hydra_calib", Hydra, self.hydra_cb)
        rospy.loginfo("Waiting for user input from the Hydra...")
        #Spin
        rate = rospy.Rate(rospy.get_param('~hz', 30))
        while not rospy.is_shutdown():
            self.draw_grippers()
            rate.sleep()

    def draw_grippers(self):
        if (self.left_target == None or self.right_target == None):
            rospy.logwarn("Waiting for the teleop controller to load before publishing markers")
            return
        #Make markers for the grippers and send them to RVIZ
        marker_msg = MarkerArray()
        lmarker = Marker()
        rmarker = Marker()
        lmarker.ns = "smart_hydra_teleop"
        rmarker.ns = "smart_hydra_teleop"
        lmarker.id = 0
        rmarker.id = 1
        lmarker.header.frame_id = "/torso_lift_link"
        rmarker.header.frame_id = "/torso_lift_link"
        #Give the marker a type
        lmarker.type = Marker.MESH_RESOURCE
        lmarker.mesh_use_embedded_materials = False
        lmarker.mesh_resource = self.grippermodel
        rmarker.type = Marker.MESH_RESOURCE
        rmarker.mesh_use_embedded_materials = False
        rmarker.mesh_resource = self.grippermodel
        lmarker.scale.x = 1.0
        lmarker.scale.y = 1.0
        lmarker.scale.z = 1.0
        rmarker.scale.x = 1.0
        rmarker.scale.y = 1.0
        rmarker.scale.z = 1.0
        #Set the colors
        lmarker.color.a = 1.0
        rmarker.color.a = 1.0
        if (self.left_state == 'F'):
            lmarker.color.r = 1.0
        elif (self.left_state == 'S'):
            lmarker.color.g = 1.0
        else:
            lmarker.color.r = 1.0
            lmarker.color.g = 1.0
        if (self.right_state == 'F'):
            rmarker.color.r = 1.0
        elif (self.right_state == 'S'):
            rmarker.color.g = 1.0
        else:
            rmarker.color.r = 1.0
            rmarker.color.g = 1.0
        #Fill in the other values
        lmarker.pose = self.left_target.pose
        rmarker.pose = self.right_target.pose
        lmarker.action = Marker.ADD
        rmarker.action = Marker.ADD
        lmarker.lifetime = rospy.Duration(1.0 / 15.0)
        rmarker.lifetime = rospy.Duration(1.0 / 15.0)
        marker_msg.markers.append(lmarker)
        marker_msg.markers.append(rmarker)
        self.marker_pub.publish(marker_msg)

    def left_gripper_state_cb(self, message):
        self.left_gripper_state = message.process_value

    def right_gripper_state_cb(self, message):
        self.latest_right_state = message.process_value

    def l_arm_joint_cb(self, msg):
        #Callback for JointTrajectoryControllerState messages from the left arm's controller
        self.current_l_arm_joints = msg.actual.positions

    def r_arm_joint_cb(self, msg):
        #Callback for JointTrajectoryControllerState messages from the right arm's controller
        self.current_r_arm_joints = msg.actual.positions

    def l_arm_pose_cb(self, pose_msg):
        #Callback for PoseStamped messages from the left arm's cartesian controller
        self.current_l_arm_pose = pose_msg.pose

    def r_arm_pose_cb(self, pose_msg):
        #Callback for PoseStamped messages from the right arm's cartesian controller
        self.current_r_arm_pose = pose_msg.pose

    def hydra_cb(self, hydra_msg):
        #Callback for hydra_calib messages from the Razer Hydra motion game controller
        if (self.BUSY):
            return
        self.BUSY = True
        #Command the base
        self.control_base(hydra_msg)
        #Command the head
        #self.control_head(hydra_msg)
        #Command the grippers
        self.control_grippers(hydra_msg)
        #Command the arms
        self.control_arms(hydra_msg)
        self.BUSY = False

    def normalize(self, input_value, scale_factor, clamp):
        #Normalize the input value to the provided limit values
        raw_value = input_value * scale_factor * clamp
        if (raw_value == -0.0):
            raw_value = 0.0
        elif (raw_value > clamp):
            raw_value = clamp
        elif (raw_value < -clamp):
            raw_value = -clamp
        return raw_value

    def control_base(self, hydra_msg):
        base_command = Twist()
        #Get the raw values and normalize them
        base_command.linear.x = self.normalize(hydra_msg.paddles[self.right].joy[self.x_axis], 1, 0.2)
        base_command.linear.y = self.normalize(-hydra_msg.paddles[self.right].joy[self.y_axis], 1, 0.2)
        base_command.angular.z = self.normalize(-(-hydra_msg.paddles[self.left].trigger + hydra_msg.paddles[self.right].trigger), 1, 0.4)
        #Check the deadman's switch
        if (ord(hydra_msg.paddles[self.left].buttons[self.deadman_button]) == 1):
            self.base_pub.publish(base_command)
        else:
            base_command.linear.x = 0.0
            base_command.linear.y = 0.0
            base_command.angular.z = 0.0
            self.base_pub.publish(base_command)

    def compute_left_arm(self, target_posestamped):
        if (target_posestamped == None):
            return None
        solution = None
        if (self.safe):
            solution = self.left_arm_kinematics.RunConstraintAwareIK(target_posestamped)
        else:
            solution = self.left_arm_kinematics.RunIK(target_posestamped)
        if (solution == None):
            rospy.logerr("Unable to find an IK solution for the left arm")
            self.left_state = 'F'
            return None
        else:
            rospy.loginfo("Found an IK solution for the left arm")
            self.left_state = 'S'
            return solution 

    def compute_right_arm(self, target_posestamped):
        if (target_posestamped == None):
            return None
        solution = None
        if (self.safe):
            solution = self.right_arm_kinematics.RunConstraintAwareIK(target_posestamped)
        else:
            solution = self.right_arm_kinematics.RunIK(target_posestamped)
        if (solution == None):
            rospy.logerr("Unable to find an IK solution for the right arm")
            self.right_state = 'F'
            return None
        else:
            rospy.loginfo("Found an IK solution for the right arm")
            self.right_state = 'S'
            return solution

    def execute_arms(self, left_solution, right_solution, wait=False):
        if (left_solution is not None and right_solution is not None):
            left_traj = self.build_trajectory_to(left_solution, self.current_l_arm_joints, self.left_arm_kinematics.ik_solver_res.kinematic_solver_info.joint_names)
            right_traj = self.build_trajectory_to(right_solution, self.current_r_arm_joints, self.right_arm_kinematics.ik_solver_res.kinematic_solver_info.joint_names)
            left_traj_goal = JointTrajectoryGoal()
            left_traj_goal.trajectory = left_traj
            right_traj_goal = JointTrajectoryGoal()
            right_traj_goal.trajectory = right_traj
            self.left_arm_client.send_goal(left_traj_goal)
            self.right_arm_client.send_goal(right_traj_goal)
            if (wait):
                self.left_arm_client.wait_for_result()
                self.right_arm_client.wait_for_result()
        if (left_solution is not None):
            left_traj = self.build_trajectory_to(left_solution, self.current_l_arm_joints, self.left_arm_kinematics.ik_solver_res.kinematic_solver_info.joint_names)
            traj_goal = JointTrajectoryGoal()
            traj_goal.trajectory = left_traj
            self.left_arm_client.send_goal(traj_goal)
            if (wait):
                self.left_arm_client.wait_for_result()
        if (right_solution is not None):
            right_traj = self.build_trajectory_to(right_solution, self.current_r_arm_joints, self.right_arm_kinematics.ik_solver_res.kinematic_solver_info.joint_names)
            traj_goal = JointTrajectoryGoal()
            traj_goal.trajectory = right_traj
            self.right_arm_client.send_goal(traj_goal)
            if (wait):
                self.right_arm_client.wait_for_result()

    def euclidean_distance(self, state1, state2):
        assert(len(state1) == len(state2))
        total = 0.0
        for index in range(len(state1)):
            temp = (state1[index] - state2[index]) ** 2
            total = total + temp
        return math.sqrt(total)

    def build_trajectory_to(self, target_joint_state, current_joint_state, joint_names):
        if (current_joint_state == None):
            rospy.logerr("No state received from the joint trajectory controller!")
            return None
        else:
            dist = self.euclidean_distance(target_joint_state, current_joint_state)
            execution_time = dist * self.execution_scaling_time
            #Assemble the trajectory
            new_traj = JointTrajectory()
            new_traj.joint_names = joint_names
            target_point = JointTrajectoryPoint()
            target_point.positions = target_joint_state
            target_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            target_point.time_from_start = rospy.Duration(execution_time)
            new_traj.points = [target_point]
            return new_traj

    def sanitize_gripper(self, close_btn, open_btn, current_state):
        #Compute the correct new value for the gripper
        if (close_btn == 1 and open_btn == 1):
            #Catch the illegal state
            return current_state
        elif (close_btn == 1):
            #Increment the gripper closer
            new_gripper = current_state - self.gripper_step
            if (new_gripper < 0.0):
                new_gripper = 0.0
            return new_gripper
        elif (open_btn == 1):
            #Increment the gripper opener
            new_gripper = current_state + self.gripper_step
            if (new_gripper > 0.08):
                new_gripper = 0.08
            return new_gripper
        else:
            return current_state

    def control_grippers(self, hydra_msg):
        #Update gripper settings
        self.left_gripper = self.sanitize_gripper(ord(hydra_msg.paddles[self.left].buttons[self.left_close]), ord(hydra_msg.paddles[self.left].buttons[self.left_open]), self.left_gripper)
        self.right_gripper = self.sanitize_gripper(ord(hydra_msg.paddles[self.right].buttons[self.right_close]), ord(hydra_msg.paddles[self.right].buttons[self.right_open]), self.right_gripper)
        #Command grippers
        l_gripper_goal = Pr2GripperCommandGoal()
        l_gripper_goal.command.position = self.left_gripper
        l_gripper_goal.command.max_effort = 50.0
        r_gripper_goal = Pr2GripperCommandGoal()
        r_gripper_goal.command.position = self.right_gripper
        r_gripper_goal.command.max_effort = 50.0
        #Send the commands
        self.left_gripper_client.send_goal(l_gripper_goal)
        self.right_gripper_client.send_goal(r_gripper_goal)

    def control_head(self, hydra_msg):
        #Setup the message components
        headGoal = PointHeadGoal()
        p = PointStamped()
        p.header.frame_id = "base_link"
        headGoal.pointing_frame = self.head_pointing_frame
        #Get raw values from the controller
        pan_axis = hydra_msg.paddles[self.left].joy[self.y_axis]
        tilt_axis = hydra_msg.paddles[self.left].joy[self.x_axis]
        #Check deadman's switch
        if (ord(hydra_msg.paddles[self.right].buttons[self.deadman_button]) == 1):
            self.head_pan = self.head_pan - pan_axis
            self.head_pan = self.normalize((self.head_pan / 120.0), 1.0, 120.0)
            self.head_tilt = self.head_tilt - tilt_axis
            self.head_tilt = self.normalize((self.head_tilt / 30.0), 1.0, 30.0)
        #Figure out actual values
        p.point.x = 1.2 + (5.0 * math.tan(self.head_tilt * (math.pi / 180.0)))
        p.point.y = 1.2 + (5.0 * math.tan(self.head_tilt * (math.pi / 180.0)))
        p.point.z = 1.2 + (5.0 * math.tan(self.head_tilt * (math.pi / 180.0)))
        #Assemble goal
        headGoal.target = p
        headGoal.min_duration = rospy.Duration(0.1)
        headGoal.max_velocity = 1.0
        #Send
        self.head_client.send_goal(headGoal)

    def set_arms(self):
        #Set the arms to a safe start point
        lcmd = PoseStamped()
        lcmd.header.frame_id = "/torso_lift_link"
        lcmd.pose.position.x = 0.6
        lcmd.pose.position.y = 0.2
        lcmd.pose.position.z = -0.1
        lcmd.pose.orientation.x = -0.00244781865415
        lcmd.pose.orientation.y = -0.548220284495
        lcmd.pose.orientation.z = 0.00145617884538
        lcmd.pose.orientation.w = 0.836329126239
        rcmd = PoseStamped()
        rcmd.header.frame_id = "/torso_lift_link"
        rcmd.pose.position.x = 0.6
        rcmd.pose.position.y = -0.2
        rcmd.pose.position.z = -0.1
        rcmd.pose.orientation.x = -0.00244781865415
        rcmd.pose.orientation.y = -0.548220284495
        rcmd.pose.orientation.z = 0.00145617884538
        rcmd.pose.orientation.w = 0.836329126239
        left_solution = self.compute_left_arm(lcmd)
        right_solution = self.compute_right_arm(rcmd)
        self.execute_arms(left_solution, right_solution, wait=True)
        rospy.loginfo("Arms moved to known start positions")
        return [lcmd, rcmd]

    def control_arms(self, hydra_msg):
        #Set the mode for left arm
        if (ord(hydra_msg.paddles[self.left].buttons[self.bumper]) == 1):
            self.left_mode += 1
            if (self.left_mode == 0):
                rospy.loginfo("Entering ACQUIRE mode [left]")
                self.left_hydra_state = deepcopy(hydra_msg.paddles[self.left].transform)
                self.left_state = 'U'
            elif (self.left_mode == 1):
                rospy.loginfo("Entering TRACKING mode [left]")
            else:
                self.left_mode = 1
        else:
            if (self.left_mode != -1):
                rospy.loginfo("Returning to IDLE mode [left]")
            self.left_mode = -1
        #Set the mode for right arm
        if (ord(hydra_msg.paddles[self.right].buttons[self.bumper]) == 1):
            self.right_mode += 1
            if (self.right_mode == 0):
                rospy.loginfo("Entering ACQUIRE mode [right]")
                self.right_hydra_state = deepcopy(hydra_msg.paddles[self.right].transform)
                self.right_state = 'U'
            elif (self.right_mode == 1):
                rospy.loginfo("Entering TRACKING mode [right]")
            else:
                self.right_mode = 1
        else:
            if (self.right_mode != -1):
                rospy.loginfo("Returning to IDLE mode [right]")
            self.right_mode = -1
        #Command each arm correctly
        if (self.left_mode == 1):
            self.left_target = self.calc_arm_target(hydra_msg.paddles[self.left], 0)
        if (self.right_mode == 1):
            self.right_target = self.calc_arm_target(hydra_msg.paddles[self.right], 1)
        #If selected, attempt to command each arm to its target
        left_command = None
        right_command = None
        if (ord(hydra_msg.paddles[self.left].buttons[self.stick]) == 1 and self.leftexec == 0):
            left_command = self.left_target
        if (ord(hydra_msg.paddles[self.right].buttons[self.stick]) == 1 and self.rightexec == 0):
            right_command = self.right_target
        left_solution = self.compute_left_arm(left_command)
        right_solution = self.compute_right_arm(right_command)
        self.execute_arms(left_solution, right_solution, wait=False)
        self.leftexec = ord(hydra_msg.paddles[self.left].buttons[self.stick])
        self.rightexec = ord(hydra_msg.paddles[self.right].buttons[self.stick])

    def calc_arm_target(self, paddle, arm_code):
        start_state = None
        arm_state = None
        if (arm_code == 0):
            start_state = self.left_hydra_state
            arm_state = self.left_target.pose
        elif (arm_code == 1):
            start_state = self.right_hydra_state
            arm_state = self.right_target.pose
        #Calculate position changes
        delta_damping = 500.0
        xdiff = (paddle.transform.translation.x - start_state.translation.x) / delta_damping
        ydiff = (paddle.transform.translation.y - start_state.translation.y) / delta_damping
        zdiff = (paddle.transform.translation.z - start_state.translation.z) / delta_damping
        #Set the new arm command
        cmd = PoseStamped()
        cmd.header.frame_id = "/torso_lift_link"
        cmd.pose.position.x = xdiff + arm_state.position.x
        cmd.pose.position.y = ydiff + arm_state.position.y
        cmd.pose.position.z = zdiff + arm_state.position.z
        cmd.pose.orientation.x = paddle.transform.rotation.x
        cmd.pose.orientation.y = paddle.transform.rotation.y
        cmd.pose.orientation.z = paddle.transform.rotation.z
        cmd.pose.orientation.w = paddle.transform.rotation.w
        return self.clip_to_arm_bounds(cmd, arm_code)

    def clip_to_arm_bounds(self, arm_target, arm_code):
        max_up = 0.5
        max_down = -0.8
        max_left = 1.2
        max_right = -1.2
        max_backward = -0.25
        max_forward = 1.0
        if (arm_code == 0):
            max_right = -1.0
        elif (arm_code == 1):
            max_left = 1.0
        if (arm_target.pose.position.x > max_forward):
            arm_target.pose.position.x = max_forward
        elif (arm_target.pose.position.x < max_backward):
            arm_target.pose.position.x = max_backward
        if (arm_target.pose.position.y > max_left):
            arm_target.pose.position.y = max_left
        elif (arm_target.pose.position.y < max_right):
            arm_target.pose.position.y = max_right
        if (arm_target.pose.position.z > max_up):
            arm_target.pose.position.z = max_up
        elif (arm_target.pose.position.z < max_down):
            arm_target.pose.position.z = max_down
        return arm_target

if __name__ == '__main__':
    #Get parameters
    #THIS IS SPECIFIC TO ARCHIE - NO OTHER PR2 HAS THIS TF FRAME!
    head_pointing_frame = "/head_mount_kinect_rgb_optical_frame"
    rospy.init_node('pr2_smart_hydra_teleop')
    SmartHydraTeleop(head_pointing_frame, 0.5, False)
