from gtpyhop import State
from geometry_msgs.msg import Pose, Point, Quaternion
from copy import copy

rigid = State("rigid")



rigid.screwdriver_pose = Pose(
    Point(
        x = -0.165,
        y = -0.82,
        z = -0.20
    ),
    Quaternion(
        x = -0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.screwdriver_pose2 = Pose(
    Point(
        x = -0.165,
        y = -0.82,
        z = -0.32
    ),
    Quaternion(
        x = -0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box_loc1 = Pose(
    Point(
        x = -0.139,
        y = 0.795,
        z = -0.20
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box_loc2 = Pose(
    Point(
        x = -0.139,
        y = 0.815,
        z = -0.31
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box2_loc1 = Pose(
    Point(
        x = -0.003,
        y = 0.783,
        z = -0.20
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box2_loc2 = Pose(
    Point(
        x = -0.003,
        y = 0.803,
        z = -0.31 
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box3_loc1 = Pose(
    Point(
        x = 0.103,
        y = 0.786,
        z = -0.20 
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box3_loc2 = Pose(
    Point(
        x = 0.103,
        y = 0.806,
        z = -0.31 
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)


rigid.box4_loc1 = Pose(
    Point(
        x = 0.212,
        y = 0.857,
        z = -0.20 
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box4_loc2 = Pose(
    Point(
        x = 0.212,
        y = 0.877,
        z = -0.31 
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)


rigid.handover_location = Pose(
    Point(
        x = 0.82,
        y = -0.23 ,
        z = 0.16,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)

rigid.X = Pose(
    Point(
        x = 0.55,
        y = -0.77 ,
        z = 0.046,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)


rigid.Y = copy(rigid.X)

rigid.workspace = Pose(
    Point(
        x = 0.60,
        y = -0.13 ,
        z = -0.20,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)

rigid.workspace2 = Pose(
    Point(
        x = 0.60,
        y = -0.13 ,
        z = -0.31,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)

rigid.workspaceL = Pose(
    Point(
        x = 0.60,
        y = 0.13 ,
        z = -0.20,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)

rigid.workspace2L = Pose(
    Point(
        x = 0.60,
        y = 0.13 ,
        z = -0.31,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)

rigid.tuck_poseR = Pose(
    Point(
        x = -0.1,
        y = -0.59,
        z = -0.18,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)

rigid.tuck_poseL = Pose(
    Point(
        x = -0.1,
        y = 0.57,
        z = -0.18,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)


rigid.locations = { 'table': [rigid.screwdriver_pose, rigid.screwdriver_pose2], 
                    'box_location': [rigid.box_loc1, rigid.box_loc2],
                    'box2_location': [rigid.box2_loc1, rigid.box2_loc2],
                    'box3_location': [rigid.box3_loc1, rigid.box3_loc2],
                    'box4_location': [rigid.box4_loc1, rigid.box4_loc2],
                    'exchange point': [rigid.handover_location],
                    'X': [rigid.X],
                    'Y': [rigid.Y],
                    'workspace': [rigid.workspace, rigid.workspace2],
                    'workspaceL': [rigid.workspaceL, rigid.workspace2L],
                    'tuck_positionR': [rigid.tuck_poseR],
                    'tuck_positionL': [rigid.tuck_poseL],
                    'brick1_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick2_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick3_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick4_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick5_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick6_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick7_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick8_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick9_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2]
                    } 
