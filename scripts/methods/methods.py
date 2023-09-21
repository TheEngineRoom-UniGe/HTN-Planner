import gtpyhop


def pick(state, agent, obj, loc_to, traj_client):
    if agent in state.agents and obj in state.objects and not state.holding[agent]:
        return [('choose_arm', obj, agent),
                ('transfer', agent, state.at[obj], traj_client), 
                ('grasp', agent, obj, traj_client), 
                ('transfer', agent, loc_to, traj_client)]


def exchange(state, agent1, agent2, obj, traj_client):
    if agent1 in state.agents and agent2 in state.agents and state.holding[agent1] == obj:
        if obj == 'screwdriver' or obj == 'screwdriver2':
            return[('grasp', agent2, obj, traj_client), 
                ('wait_on_condition', 'tactile', agent1, None, traj_client),
                ('release', agent1, obj, traj_client),
                ('reset_active_arm', agent1)]
        elif 'box' in obj:
            return[('grasp', agent2, obj, traj_client), 
                   ('wait_on_condition', 'baxter_camera_box', None, obj, traj_client),
                   ('transfer', agent1, state.boxes_home_pose[obj], traj_client),
                   ('release', agent1, obj, traj_client),
                   ('reset_active_arm', agent1)]


def receive(state, agent, obj, traj_client):
    if agent in state.agents and obj in state.objects:
        return [('transfer', agent, 'exchange point', traj_client)]


def handover(state, agent1, agent2, obj, traj_client):
    if agent1 in state.agents and agent2 in state.agents:
        return [('pick', agent1, obj, 'exchange point', traj_client), 
               ('receive', agent2, obj, traj_client), 
               ('exchange', agent1, agent2, obj, traj_client)]


def pick_and_place(state, agent, loc_to, traj_client):
    if state.selected_object and agent in state.agents and loc_to in state.locations:
        return [('pick', agent, state.selected_object, loc_to, traj_client), 
                ('release', agent, state.selected_object, traj_client),
                ('reset_active_arm', agent),
                ('reset_selected_object',)]


def deliver_objects(state, agent, obj_list, traj_client):
    if agent in state.agents and set(obj_list) <= state.objects and not state.holding[agent]:
        return [
                ('check_available_obj', obj_list, traj_client), 
                ('process_available_objects', obj_list, agent, traj_client)]


def do_nothing(state, obj_list, agent, traj_client):
    if state.available_objects == []:
        return []


def sub_delivery(state, obj_list, agent, traj_client):
    if state.available_objects != []:
        return [('deliver_objects', agent, obj_list, traj_client)]


def choose_and_deliver(state, obj_list, agent, traj_client):
    if state.available_objects != []:
        return [('choose_obj',), 
                ('pick_and_place', agent, 'workspace', traj_client), 
                ('continue_delivery', obj_list, agent, traj_client)]
    

def assembly_chair(state, client):
    if state.goal_object == 'chair':
        return  [
                ('handover', 'robot', 'human', 'box', client),
                ('deliver_objects', 'robot', ['brick1', 'brick2', 'brick3', 'brick4'], client), #ids 0, 1, 20, 200
                ('deliver_objects', 'robot',  ['brick5'], client), # id 100
                ('handover', 'robot', 'human', 'box3', client),
                ('handover', 'robot', 'human', 'box4', client),
                ('deliver_objects', 'robot', ['brick6'], client), # id 10
                ('wait_on_condition', 'imu', None, None, client),
                ('handover', 'robot', 'human', 'box2', client),
                ('handover', 'robot', 'human', 'screwdriver', client),
                ]


def assembly_bottle_holder(state, client):
    if state.goal_object == 'bottle_holder':
        return  [
                ('deliver_objects', 'robot', ['brick5'], client), # id 100
                ('deliver_objects', 'robot', ['brick1', 'brick2', 'brick3'], client), # ids 0, 1, 20
                ('handover', 'robot', 'human', 'box', client),
                ('deliver_objects', 'robot', ['brick6', 'brick4'], client), # ids 10, 200
                ('wait_on_condition', 'imu', None, None, client),
                ('deliver_objects', 'robot', ['brick7', 'brick8', 'brick9'], client), # ids 2, 4, 40
                ('handover', 'robot', 'human', 'box2', client),
                ]


def assembly_child_chair(state, client):
    if state.goal_object == 'child_chair':
        return [
                ('handover', 'robot', 'human', 'screwdriver', client),
                ('handover', 'robot', 'human', 'box', client),
                ('deliver_objects', 'robot', ['brick1', 'brick3'], client), # ids 0, 1
                ('deliver_objects', 'robot', ['brick2'], client), # marker 20
                ('deliver_objects', 'robot', ['brick4'], client), # marker 200
                ('deliver_objects', 'robot', ['brick5'], client), # marker 100
                ('wait_on_condition', 'imu', None, None, client),
                ('deliver_objects', 'robot', ['brick6'], client), # marker 10
                ('handover', 'robot', 'human', 'box2', client),
                ]


def assembly_paper_holder(state, client):
    if state.goal_object == 'paper_holder':
        return [
                ('deliver_objects', 'robot', ['brick5'], client), # brick1 is 1st side of the paper holder
                ('handover', 'robot', 'human', 'box', client),
                ('deliver_objects', 'robot', ['brick2', 'brick3', 'brick4', 'brick1'], client),
                ('wait_on_condition', 'imu', None, None, client),
                ('deliver_objects', 'robot', ['brick6'], client), # brick7 is 2nd side of the paper holder
                ]


def loop(state, traj_client):
    if True:
        return [
            ('define_goal',), 
            ('wait_on_condition', 'imu', None, None, traj_client),
            ('assembly', traj_client), 
            ('reset_goal',), 
            ('loop', traj_client)]


gtpyhop.declare_task_methods('pick', pick)
gtpyhop.declare_task_methods('receive', receive)
gtpyhop.declare_task_methods('exchange', exchange)

gtpyhop.declare_task_methods('handover', handover)

gtpyhop.declare_task_methods('process_available_objects', choose_and_deliver, do_nothing)
gtpyhop.declare_task_methods('continue_delivery', sub_delivery, do_nothing)
gtpyhop.declare_task_methods('pick_and_place', pick_and_place)
gtpyhop.declare_task_methods('deliver_objects', deliver_objects)
gtpyhop.declare_task_methods('assembly', assembly_chair, assembly_bottle_holder,assembly_child_chair,assembly_paper_holder)

gtpyhop.declare_task_methods('loop', loop)
