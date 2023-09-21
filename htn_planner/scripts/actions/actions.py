import gtpyhop
import time 
import random
import rospy
from state.rigid import rigid
import copy


def transfer(state, agent, loc_to, traj_client):
    if agent in state.agents and loc_to in state.locations:
        loc_from = state.at[agent]

        if 'workspace' == loc_from and state.active_arm[agent] == 'left':
            loc_from += 'L'
        if 'workspace' == loc_to and state.active_arm[agent] == 'left':
            loc_to += 'L'
        loc_to_pose = copy.deepcopy(rigid.locations[loc_to])

        if agent == 'robot':
            if loc_to == 'exchange point' and state.active_arm[agent] == 'left':
                for i in range(len(loc_to_pose)):
                    loc_to_pose[i].position.y *= -1.0
            if len(rigid.locations[loc_from])>1:
                if not traj_client.transfer([rigid.locations[loc_from][0]], state.active_arm[agent]):
                    return state
            if not traj_client.transfer(loc_to_pose, state.active_arm[agent]):
                return state
            
            if state.selected_object:
                if 'brick' in state.selected_object and 'workspace' not in loc_to: #loc_to != 'workspace':
                    print('selected_object: ', state.selected_object)
                    selected_marker_id = int(state.obj2markerID[state.selected_object])
                    precise_pose = precision_marker_detection(state, traj_client, selected_marker_id, state.active_arm[agent])
                    print(precise_pose)
                    rigid.locations[state.selected_object + '_pose'].append(precise_pose)
                    print('new location for ', state.selected_object, ' is: ', rigid.locations[state.selected_object + '_pose'])
                    if not traj_client.transfer([precise_pose], state.active_arm[agent]):
                        return state

        state.at[agent] = loc_to
        state.at[state.active_arm[agent]] = loc_to
        if state.holding[agent]:
            obj = state.holding[agent]
            state.at[obj] = loc_to
        return state
    

def tuck_arms(state, agent, traj_client):
    if agent in state.agents and not state.holding[agent]:
        state.at[agent] = 'workspace' 
        state.at['left'] = 'workspaceL' 
        state.at['right'] = 'workspace' 

        if not traj_client.transfer([rigid.locations['workspace'][0]], 'right'):
            return None 

        if not traj_client.transfer([rigid.locations['workspaceL'][0]], 'left'):
            return None
    return state


def grasp(state, agent, obj, traj_client):
    if agent in state.agents and obj in state.objects and state.at[agent] == state.at[obj]: #and not state.holding[agent]:
        if agent == 'robot':
            if not state.holding[agent]:
                traj_client.traj_p.close_gripper(state.active_arm[agent])
                state.holding[agent] = obj
        if agent == 'human':
            if obj == 'box':
                state.holding[agent] = None
            else:      
                state.holding[agent] = obj
        return state
    

def precision_marker_detection(state, traj_client, id, side):
    print('precision_marker_detection')
    traj_client.baxter_camera_activation_pub.publish(f'{id}_{side}')

    print('activating baxter camera with id: ', id)
    if traj_client.stop_sleeping_sig.is_set():
        traj_client.stop_sleeping_sig.clear()
    traj_client.stop_sleeping_sig.wait()
    return traj_client.precise_m_p


def empty_box_detection(state, obj, traj_client):
    if state.box_empty[obj] == False:

        traj_client.camera_activation_pub.publish(state.active_arm['robot'])

        if traj_client.stop_sleeping_sig.is_set():
            traj_client.stop_sleeping_sig.clear()
        traj_client.stop_sleeping_sig.wait()
        state.box_empty[obj] = True
        return state
    

def wait_on_condition(state, perception_module, agent, object, traj_client):
    if perception_module == 'tactile':
        tool_pulling_detection(state, agent, traj_client)
    elif perception_module == 'imu':
        idle_detection(state, traj_client)
    elif perception_module == 'baxter_camera_box':
        empty_box_detection(state, object, traj_client)

def tool_pulling_detection(state, agent, traj_client):
    if agent in state.agents:
        if agent == 'robot':
            if state.active_arm[agent] == 'right':
                traj_client.melexis_activation_pub.publish(True)
                if traj_client.stop_sleeping_sig.is_set():
                    traj_client.stop_sleeping_sig.clear()
                traj_client.stop_sleeping_sig.wait()
                traj_client.traj_p.open_gripper('right')
            elif state.active_arm[agent] == 'left':
                traj_client.traj_p.open_gripper('left')
    return state


def idle_detection(state, traj_client):
    traj_client.idle_classification_pub.publish(True)
    if traj_client.stop_sleeping_sig.is_set():
        traj_client.stop_sleeping_sig.clear()
    traj_client.stop_sleeping_sig.wait()
    print('not idle anymore')
    return state


def release(state, agent, obj, traj_client):
    if agent in state.agents and obj in state.objects and state.holding[agent] == obj:
        if agent == 'robot':
            traj_client.traj_p.open_gripper(state.active_arm[agent])
        state.holding[agent] = None
        return state 


def check_available_obj(state, obj_list, traj_client):
    if set(obj_list) <= state.objects:
        # use camera to check what objects are available
        traj_client.aruco_activation_pub.publish(True)
        if traj_client.stop_sleeping_sig.is_set():
            traj_client.stop_sleeping_sig.clear()
        traj_client.stop_sleeping_sig.wait()
        if len(traj_client.current_obj) == 1 and traj_client.current_obj[0] == '':
            traj_client.current_obj = []
            traj_client.active_marker_poses = {}
        
        state.available_objects = []

        print('available markers: ', traj_client.current_obj)
        print('active_marker_poses: ', traj_client.active_marker_poses)
        print('state.available_objects: ', state.available_objects)
        for obj_marker_id in traj_client.current_obj:
            if state.markerID2obj[obj_marker_id] not in state.available_objects and state.markerID2obj[obj_marker_id] in obj_list:
                state.available_objects.append(state.markerID2obj[obj_marker_id])
            rigid.locations[state.markerID2obj[obj_marker_id] + '_pose'] = [traj_client.active_marker_poses[obj_marker_id]]
            state.at[state.markerID2obj[obj_marker_id]] = state.markerID2obj[obj_marker_id] + '_pose'

        print('available objects: ', state.available_objects)
        print('obj2pose: ', rigid.locations)
        return state


def choose_obj(state):
    if state.available_objects:
        length = len(state.available_objects)
        if length == 0:
            state.selected_object = None
        elif length == 1:
            state.selected_object = state.available_objects[0]
        else:
            state.selected_object = random.choice(state.available_objects)
        return state


def choose_arm(state, obj, agent):
    if agent in state.agents and obj in state.objects:
        if agent == 'human':
            return state
        elif agent == 'robot':
            loc = state.at[obj]
            if rigid.locations[loc][0].position.y > 0:
                state.active_arm[agent] = 'left'
            else:
                state.active_arm[agent] = 'right'
            state.at[agent] = state.at[state.active_arm[agent]]
        return state
    

def reset_active_arm(state, agent):
    if agent in state.agents:
        if agent == 'human':
            return state
        elif agent == 'robot':
            state.active_arm[agent] = None
        return state
    

def reset_selected_object(state):
    state.selected_object = None
    return state


def define_goal(state):
    if state.goal_object == None:
        object = None
        while object not in ['chair', 'child_chair', 'bottle_holder', 'paper_holder', 'stop']:
            object = input('Please enter the object you want to build: \n Possible objects: chair, child_chair, bottle_holder, paper_holder or stop to stop \n')

        state.goal_object = object
        return state


def reset_goal(state):
    if state.goal_object != None:
        state.goal_object = None
        state.at['screwdriver'] = 'table'
        for b in ['box', 'box2', 'box3', 'box4']:
            state.box_empty[b] = False
            state.at[b] = state.boxes_home_pose[b]
        return state


gtpyhop.declare_actions(transfer, 
                        grasp, 
                        release, 
                        tool_pulling_detection, 
                        wait_on_condition,
                        empty_box_detection,
                        idle_detection, 
                        check_available_obj,
                        choose_obj, 
                        choose_arm, 
                        reset_active_arm, 
                        reset_selected_object,
                        tuck_arms,
                        define_goal,
                        reset_goal)