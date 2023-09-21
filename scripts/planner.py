#!/usr/bin/env python

import sys
import gtpyhop
import rospy
from joint_trajectory_client import TrajectoryClient
import baxter_interface
from baxter_interface import CHECK_VERSION
import argparse
from zed2_pose_estimation import ArucoDetection
from geometry_msgs.msg import Pose
domain = gtpyhop.Domain('handover')
from methods.methods import *
from actions.actions import *
from state.rigid import rigid

sys.setrecursionlimit(3000)
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
    client.traj_p.close_gripper('left')
    client.traj_p.close_gripper('right')
    # client.transfer(rigid.locations['table'], 'right')
    # exit()
    # wait_idle(state1, client)
    # result = gtpyhop.find_plan(state1, [
    #                                     # ('wait_idle', client),
    #                                     # ('tuck_arms', 'robot', client),
    #                                     ('handover', 'robot', 'human', 'box', client),
    #                                     ('handover', 'robot', 'human', 'box2', client),
    #                                     ('deliver_objects', 'robot', ['brick1', 'brick2', 'brick3', 'brick4'], client),
    #                                     ('deliver_objects', 'robot', ['brick5', 'brick6'], client),
    #                                     ('handover', 'robot', 'human', 'screwdriver', client),
    #                                     ('handover', 'robot', 'human', 'box3', client),
    #                                     ])
    result = gtpyhop.find_plan(state1, [('loop', client)])
    return


    # result = gtpyhop.find_plan(state1, [('handover', 'robot', 'human', 'box', client)])
    # result = gtpyhop.find_plan(state1, [('handover', 'robot', 'human', 'screwdriver', client)])
    # result = gtpyhop.find_plan(state1, [('deliver_objects', 'robot', ['brick1', 'brick2', 'brick3'], client)])
    exit()
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
    HD = ArucoDetection()

    tuck_pose = Pose()
    tuck_pose.position.x = 0.38
    tuck_pose.position.y = -0.43
    tuck_pose.position.z = -0.20
    tuck_pose.orientation.x = 1.0
    tuck_pose.orientation.y = 0.0
    tuck_pose.orientation.z = 0.0
    tuck_pose.orientation.w = 0.0
    client.transfer([tuck_pose], 'right')
    # exit()  
    while True:
        dict = HD.loop()
        id = random.choice(list(dict.keys()))
        print(dict)
        print(f'ID: {id}')
        t, r = dict[id]
        # t[2] += 0.04
        print(f' trasl: {t} \nrot: {r}')
        # input("Press Enter to continue...")
        if t is not None and r is not None:
            p = Pose()
            p.position.x = float(f'{t[0]:.3f}')
            p.position.y = float(f'{t[1]:.3f}')
            p.position.z = float(f'{t[2]:.3f}')
            if p.position.z < -0.32:
                p.position.z = -0.32
            p.position.z += 0.08
            p.orientation.x = 1.0 #float(r.x)
            p.orientation.y = 0.0 #float(r.y)
            p.orientation.z = 0.0 #float(r.z)
            p.orientation.w = 0.0 #float(r.w)
            client.transfer([p], 'right')
            pmp = precision_marker_detection(state1, client, id)
            pmp.position.z -= 0.06
            if pmp.position.z < -0.32:
                pmp.position.z = -0.32
            rospy.logwarn(pmp)
            pmp.orientation.x = 1.0
            pmp.orientation.y = 0.0
            pmp.orientation.z = 0.0
            pmp.orientation.w = 0.0
            # input("Press Enter to continue...")
            client.transfer([pmp], 'right')
            client.transfer([p], 'right')
            client.transfer([tuck_pose], 'right')
            # input("Press Enter to continue...")
    exit()
    time.sleep(1)
    # result = gtpyhop.find_plan(state1, [('handover', 'robot', 'human', 'screwdriver', client)])
    result = gtpyhop.find_plan(state1, [('handover', 'robot', 'human', 'box', client), ('handover', 'robot', 'human', 'screwdriver', client)])
    # result = gtpyhop.find_plan(state1, [('deliver_objects', 'robot', ['brick1', 'brick6', 'brick4', 'brick3', 'brick2', 'brick5'], client)])

    print(result)

if __name__ == "__main__":
    main()
