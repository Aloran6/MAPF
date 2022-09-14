import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
 
    table = {}
    positive_table = {}
    locked = []
    for constraint in constraints:
        if isinstance(constraint,tuple):
            locked.append(constraint) #this is a goal location, used for prioritizied method
            continue
        if 'positive' not in constraint.keys(): #a negative constraint
            constraint['positive'] = False
        if constraint['agent'] == agent:
            if constraint['positive']: #add to positive table
                if constraint['time_step'] in positive_table.keys():
                    positive_table[constraint['time_step']].append(constraint['loc'])
                else: 
                    positive_table[constraint['time_step']] = [constraint['loc']]
            else: #add to negative table
                if constraint['time_step'] in table.keys():
                    table[constraint['time_step']].append(constraint['loc'])
                else: 
                    table[constraint['time_step']] = [constraint['loc']]        
    return table, locked, positive_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
  
    if next_time in constraint_table:
        for i in constraint_table[next_time]:
            if i == [next_loc] or i == [curr_loc, next_loc]:
                return True
    
    return False

def passes_positive_constraints(curr_loc, next_loc, next_time, constraint_table):
    #added for the purpoase of checking if the positive constraint has been satisfied
    #True = GOOD, False = BAD
    if next_time in constraint_table:
            for i in constraint_table[next_time]:
                if i == [next_loc] or i == [curr_loc, next_loc]:
                    return True
    else: #if not in the constraint table, meaning there is no positive constraint
        return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each time_step
    """

    table, locked, pos_table = build_constraint_table(constraints,agent)
    #table - negative constraints, pos_table - positive constraints, locked - "locked" location of previou agents' goal locations (used for prioritized method)
    open_list = []
    closed_list = dict()
   
    if table.keys(): #if we have a constraint table, the ealiest finish time should be later than the very first agent
        earliest_goal_time_step = max(table.keys())
    else:
        earliest_goal_time_step = 0

    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0}
    push_node(open_list, root)
    closed_list[(root['loc'],root['time_step'])] = root

    #2.4 Limits the amount of time for path finding
    map_slots = len(my_map)*len(my_map[0])
    upperLimit = map_slots + earliest_goal_time_step

    while len(open_list) > 0:

        curr = pop_node(open_list)
        if curr['time_step'] > upperLimit: #took too long
            break
       
        if curr['loc'] == goal_loc and curr['time_step'] >= earliest_goal_time_step: #curr['time_step'] >= 10
            return get_path(curr)

        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            #manual map constraint for test cases since there is no border
            # if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] > 7 or child_loc[1] > 7 :
            #     continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr['loc'],child_loc,curr['time_step']+1, table): #negative constraint needs to be FALSE
                continue
            if not passes_positive_constraints(curr['loc'],child_loc,curr['time_step']+1, pos_table): #positive constraint needs to be TRUE
                continue
           
            #FOR PRIORITIZED GOAL LOCKING, 'lock' the goal location of previous agents so that future agents cannot go there
            if child_loc in locked:
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'time_step': curr['time_step']+1}
            
            if (child['loc'],child['time_step']) in closed_list:
                existing_node = closed_list[(child['loc'],child['time_step'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'],child['time_step'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'],child['time_step'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
