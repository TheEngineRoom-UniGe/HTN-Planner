#!/usr/bin/env python

import argparse
import sys
from threading import Event
import signal
from copy import copy, deepcopy
import rospy
import actionlib
from time import sleep
import baxter_interface
from trajectoryPlanner import TrajectoryPlanner
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    GripperCommandActionGoal
)
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from geometry_msgs.msg import Pose, PoseArray

from baxter_moveit.msg import MoveitTrajectory

from baxter_interface import CHECK_VERSION

# Client class to provides trajectory execution service
class TrajectoryClient():

    def __init__(self, limb):
        self.limb = limb
        rospy.Subscriber("/right_arm/baxter_moveit_trajectory", MoveitTrajectory, self.callback)
        self.release_sub = rospy.Subscriber("/melexis_release", Bool, self.release_callback)
        self.aruco_objects = rospy.Subscriber("/aruco_detection", PoseArray, self.aruco_callback)
        self.precision_marker_pose = rospy.Subscriber("/precise_marker_pose", Pose, self.precise_marker_pose_callback)
        self.idle_classification_sub = rospy.Subscriber("/IMU_HAR_idle", Bool, self.IMU_HAR_idle_callback)

        self.melexis_activation_pub = rospy.Publisher("/melexis_activation", Bool, queue_size=10)
        self.camera_activation_pub = rospy.Publisher("/camera_listener_activation", String, queue_size=10)
        self.baxter_camera_activation_pub = rospy.Publisher("/baxter_camera_listener_activation", String, queue_size=10)
        self.aruco_activation_pub = rospy.Publisher("/aruco_detection_activation", Bool, queue_size=10)
        self.idle_classification_pub = rospy.Publisher("/IMU_HAR_activation", Bool, queue_size=10)

        self.trajTime = 3.5 # Time to complete each trajectory 
        self.traj_p = TrajectoryPlanner()
        self.stop_sleeping_sig = Event()

        self.current_obj = []
        self.active_marker_poses = {}
        self.precise_m_p = None

    def callback(self, msg):
        trajectory = msg.trajectory
        n = len(trajectory)

        for i in range(len(trajectory)):
            traj = Trajectory(self.limb)

            rospy.on_shutdown(traj.stop)
            # Command Current Joint Positions first
            limb_interface = baxter_interface.limb.Limb(self.limb)

            t = 0
            for point in trajectory[i].joint_trajectory.points:
  
                t += self.trajTime/len(trajectory[i].joint_trajectory.points)      #prima c'era 0.1
                traj.add_point(point.positions, point.velocities, point.accelerations, t)
                
            traj.start()
            traj.wait()
        
        self.ex_complete_pub.publish(True)
        print("Action Complete")

    def IMU_HAR_idle_callback(self, msg):
        rospy.loginfo("IMU_HAR_idle_callback called")
        print(msg)
        if not self.stop_sleeping_sig.is_set():
            self.stop_sleeping_sig.set()


    def precise_marker_pose_callback(self, msg):
        rospy.loginfo("precise_marker_pose_callback called")
        # print(msg)
        self.precise_m_p = msg
        if not self.stop_sleeping_sig.is_set():
            self.stop_sleeping_sig.set()

    def aruco_callback(self, msg):
        rospy.loginfo("aruco_callback")
        print(msg)

        if msg.header.frame_id == '':
            self.current_obj = ['']
            self.active_marker_poses = {}
            if not self.stop_sleeping_sig.is_set():
                self.stop_sleeping_sig.set()

        else:    
            self.current_obj = msg.header.frame_id.split('_')
            for i, mid in enumerate(self.current_obj):
                self.active_marker_poses[mid] = msg.poses[i]
            if not self.stop_sleeping_sig.is_set():
                self.stop_sleeping_sig.set()

    def release_callback(self, msg):
        rospy.loginfo("Release callback")
        if not self.stop_sleeping_sig.is_set():
            self.stop_sleeping_sig.set()
        if msg.data:
            self.traj_p.open_gripper('right')

    def execute_trajectory(self, trajectory, side, speed):

        traj = Trajectory(side)

        rospy.on_shutdown(traj.stop)
        # Command Current Joint Positions first
        # limb_interface = baxter_interface.limb.Limb(self.limb)

        t = 0
        counter = 0
        avg = self.trajTime/len(trajectory.joint_trajectory.points)
        third = len(trajectory.joint_trajectory.points) / 5
        for point in trajectory.joint_trajectory.points:

            if counter < third:
                t += (1.5*avg)-(counter * avg / (2*third))

            elif counter >= third and counter < 4 * third:
                t += avg

            elif counter >= 4 * third:
                t += avg + (counter * avg / (2*third))
            # t += avg     #prima c'era 0.1
            traj.add_point(point.positions, point.velocities, point.accelerations, t)
            
            counter += 1
            
        traj.start()
        traj.wait()
    
        print("Action Complete")

    def transfer(self, poses, side):
        failures = False
        first = True

        for pose in poses:
            rospy.logerr("Transfer function called")
            rospy.logerr(pose)
            rospy.logerr(side)
            temp_pose = deepcopy(pose)
            #reach XY location above object
            simple_traj, fract = self.traj_p.plan_cartesian_trajectory(temp_pose, side)
            
            print('fraction', fract)
            if fract > 0.80:
                if first:
                    self.execute_trajectory(simple_traj, side,2.5)
                else:
                    self.execute_trajectory(simple_traj, side,3.0)
            else:
                rospy.logerr(fract)
                failures = True
        return not failures
            # input("Press Enter to continue...")

        # #remove height offset once you reach XY location
        # temp_pose.position.z -= 0.1
        # simple_traj, fract = self.traj_p.plan_cartesian_trajectory(temp_pose)
        # print('fraction', fract)
        # if fract > 0.99:
        #     self.execute_trajectory(simple_traj)        

    def tuck(self):
        # current_joint_state = JointState()
        for limb in ['left', 'right']:
            if limb == 'left':
                m_group = self.traj_p.move_group_left
                joint_values = [-0.10392719837923678, 2.1563934925699204, 1.495247772991307, -0.6715000898968398, -0.7923010769428162, -0.07746602978821339, 3.051087787104088]
            else:
                m_group = self.traj_p.move_group_right
                joint_values = [-0.10316020798529407, 2.223505152039907, -1.4891118498397653, -0.6550097964270717, -2.2422964166915036, -0.11083011192472114, 3.017340209770609]
            joint_names = [limb + '_' + joint for joint in ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']]
            # current_joint_state.name = joint_names
            # current_joint_state.position = start_joint_config
            # moveit_robot_state = RobotState()
            # moveit_robot_state.joint_state = current_joint_state
            # self.move_group.set_start_state(moveit_robot_state)
            dict = {}
            for name, value in zip(joint_names, joint_values):
                dict[name] = value
            m_group.set_joint_value_target(dict)
            m_group.set_goal_tolerance(10e-3)
            
            plan = m_group.plan()
            self.execute_trajectory(plan[1], limb)


    def reach_handover_location(self):
        pose = Pose()
        pose.position.x = 0.82
        pose.position.y = -0.13
        pose.position.z = 0.16
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        simple_traj, fract = self.traj_p.plan_cartesian_trajectory(pose)
        print('fraction', fract)
        if fract > 0.90:
            self.execute_trajectory(simple_traj)



    def handover(self, pose, mode):
        self.transfer(pose)
            #grasp object
        self.traj_p.close_gripper()
        self.reach_handover_location()
        if mode == 'tool':
            self.melexis_activation_pub.publish(True)
        elif mode == 'box':
            self.camera_activation_pub.publish(True)
        print('start sleeping')
        # if self.stop_sleeping_sig.is_set():
        #     self.stop_sleeping_sig.clear()
        # self.stop_sleeping_sig.wait()
        sleep(3)
        print('stop sleeping')



# Trajectory class to handle joint trajectory action
class Trajectory(object):

    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.05)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)
        gripper_topic = "robot/end_effector/" + limb + "_gripper/gripper_action/goal"
        self.gripper_publisher = rospy.Publisher(gripper_topic, GripperCommandActionGoal, queue_size=10)

    def add_point(self, positions, velocities, accelerations, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = copy(velocities)
        point.accelerations = copy(accelerations)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
            
    def open_gripper(self): 
        gripperCommandMsg = GripperCommandActionGoal()
        gripperCommandMsg.goal.command.position = 100.0
        self.gripper_publisher.publish(gripperCommandMsg)
    	
    def close_gripper(self):
        gripperCommandMsg = GripperCommandActionGoal()
        gripperCommandMsg.goal.command.position = 0.0
        self.gripper_publisher.publish(gripperCommandMsg)

    

def main():
    
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l',
        '--limb',
        required=True,
        choices=['left', 'right'],
        help='Send joint trajectory to which limb'
    )

    args = parser.parse_args(rospy.myargv()[1:])

    # Instantiate trajectory handler object
    client = TrajectoryClient(args.limb)

    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client_%s" % (args.limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running...")
    

    client.traj_p.open_gripper()
    screwdrivre_pose = Pose()
    screwdrivre_pose.position.x = 0.52
    screwdrivre_pose.position.y = -0.31
    screwdrivre_pose.position.z = -0.22
    screwdrivre_pose.orientation.x = 1.0
    screwdrivre_pose.orientation.y = 0.0
    screwdrivre_pose.orientation.z = 0.0
    screwdrivre_pose.orientation.w = 0.0

    while not rospy.is_shutdown():
        # input("Press Enter to continue...")
        # client.pick_object(screwdrivre_pose)
        # client.reach_handover_location()
        # client.melexis_activation_pub.publish(True)
        # print('start sleeping')
        # if client.stop_sleeping_sig.is_set():
        #     client.stop_sleeping_sig.clear()
        # client.stop_sleeping_sig.wait()
        # print('stop sleeping')
        client.handover(screwdrivre_pose, 'box')
    
    rospy.spin()


if __name__ == "__main__":
    main()
