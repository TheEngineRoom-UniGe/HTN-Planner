from gtpyhop import State

state = State("state 0")

state.goal_object = None

state.objects = {'screwdriver', 'screwdriver2', 'box', 'box2', 'box3', 'box4', 'dummy_tuck_objL', 'dummy_tuck_objR', 'screws', 
                 'brick1', 'brick2', 'brick3', 'brick4', 'brick5', 'brick6', 'brick7', 'brick8',
                 'brick9', 'shelf2', 'shelf3'}
state.box_empty = {'box': False, 'box2': False, 'box3': False, 'box4': False}
state.obj_properties = {'screwdriver': None, 'screwdriver2': None, 'box': state.box_empty['box'],
                         'box2': state.box_empty['box2'], 'box3': state.box_empty['box3']}
state.active_arm = {'human': None, 'robot': None}

state.markerID2obj = {'0': 'brick1', '20': 'brick2', '1': 'brick3', '200': 'brick4', '100': 'brick5', '10': 'brick6', 
                      '2': 'brick7', '4': 'brick8', '40': 'brick9'}
state.obj2markerID = inv_map = {v: k for k, v in state.markerID2obj.items()}

state.available_objects = []
state.selected_object = None

state.locations = {'table', 'table2', 'tuck_positionL', 'tuck_positionR', 'exchange point', 'X', 'Y', 'workspace', 'workspaceL',
                    'brick1_pose', 'brick2_pose', 'brick3_pose','brick4_pose', 'brick5_pose', 'brick6_pose', 'brick7_pose', 'brick8_pose','brick9_pose',
                    'box_location', 'box2_location', 'box3_location', 'box4_location'}
state.agents = {'human', 'robot'}

state.boxes_home_pose = {'box': 'box_location', 'box2': 'box2_location', 'box3': 'box3_location', 'box4': 'box4_location'}
state.holding = {'human': None, 'robot': None}
state.at = {'box': 'box_location', 
            'box2': 'box2_location', 
            'box3': 'box3_location', 
            'box4': 'box4_location', 
            'screwdriver': 'table', 
            'screwdriver2': 'table2', 
            'dummy_tuck_objL': 'tuck_positionL',
            'dummy_tuck_objR': 'tuck_positionR',
            'human': 'X', 
            'robot': 'Y',
            'left': 'Y',
            'right': 'Y',
            'brick1': 'brick1_pose', 
            'brick2': 'brick2_pose', 
            'brick3': 'brick3_pose', 
            'brick4': 'brick4_pose', 
            'brick5': 'brick5_pose',
            'brick6': 'brick6_pose',
            'brick7': 'brick7_pose',
            'brick8': 'brick8_pose',
            'brick9': 'brick9_pose'}
