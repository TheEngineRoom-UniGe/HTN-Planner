#!/usr/bin/env python

import gtpyhop
import rospy
from joint_trajectory_client import TrajectoryClient
import baxter_interface
from baxter_interface import CHECK_VERSION
import argparse
from logitech_pose_estimation import ArucoDetection
from geometry_msgs.msg import Pose

domain = gtpyhop.Domain('handover')
from methods.methods import *
from actions.actions import *
from state.rigid import rigid

def main():

    gtpyhop.current_domain = domain

    from state.state import state

    state.display('This is initial state')

    state1 = state.copy()
    # state1.display('This is state1')
    gtpyhop.verbose = 3

    gtpyhop.print_domain()

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
    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client_%s" % (args.limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running...")
    client = TrajectoryClient(args.limb)    

    # client.traj_p.close_gripper('left')
    # client.traj_p.close_gripper('right')
    # time.sleep(3)
    client.traj_p.open_gripper('left')
    client.traj_p.open_gripper('right')

    # result = gtpyhop.find_plan(state1, [('handover', 'robot', 'human', 'box', client)])
    result = gtpyhop.find_plan(state1, [('handover', 'robot', 'human', 'screwdriver', client)])
    # result = gtpyhop.find_plan(state1, [('handover', 'robot', 'human', 'screwdriver', client), ('handover', 'robot', 'human', 'screwdriver2', client)])

    # iter = 0
    # while True:
    #     result = gtpyhop.find_plan(state1, [('handover', 'robot', 'human', 'screwdriver', client), ('handover', 'robot', 'human', 'screwdriver2', client)])
    #     # client.transfer(rigid.locations['exchange point'])
    #     time.sleep(1)
    #     side = None 
    #     if rigid.locations['test'][0].position.y > 0.0:
    #         side = 'left'
    #     else:
    #         side = 'right'

    #     client.transfer(rigid.locations['test'], side)
    #     time.sleep(2)
    #     # rigid.handover_location.position.y *= -1.0
    #     rigid.test.position.y *= -1.0
    #     rigid.screwdriver_pose.position.y *= -1.0
    #     rigid.screwdriver_pose2.position.y *= -1.0
    #     rigid.handover_location.position.y *= -1.0
    #     iter += 1
    #     if iter == 4:
    #         break
    # HD = ArucoDetection()
    # while True:
    #     t,r = HD.loop()
    #     if t is not None and r is not None:
    #         p = Pose()
    #         p.position.x = float(f'{t[0]:.3f}')
    #         p.position.y = float(f'{t[1]:.3f}')
    #         p.position.z = -0.30#t[2]
    #         p.orientation.x = 1.0 #float(r.x)
    #         p.orientation.y = 0.0 #float(r.y)
    #         p.orientation.z = 0.0 #float(r.z)
    #         p.orientation.w = 0.0 #float(r.w)
    #         client.transfer([p])
    #         input("Press Enter to continue...")
    exit()
    time.sleep(1)
    # result = gtpyhop.find_plan(state1, [('handover', 'robot', 'human', 'screwdriver', client)])
    result = gtpyhop.find_plan(state1, [('handover', 'robot', 'human', 'box', client), ('handover', 'robot', 'human', 'screwdriver', client)])
    # result = gtpyhop.find_plan(state1, [('deliver_objects', 'robot', ['brick1', 'brick6', 'brick4', 'brick3', 'brick2', 'brick5'], client)])

    print(result)

if __name__ == "__main__":
    main()
