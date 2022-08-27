import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, push_node


def detect_collision(path1, path2):
    
    for i in range(max(len(path1),len(path2))):
            path1_loc = get_location(path1,i)
            path2_loc = get_location(path2,i)

            if path1_loc == path2_loc:
                return {'loc':[path1_loc], 'time_step':i}
            if i > 0:
                path1_prev_loc = get_location(path1, i-1)
                path2_prev_loc = get_location(path2, i-1)
                
                if path1_loc == path2_prev_loc and path1_prev_loc == path2_loc:
                    return {'loc':[path1_loc,path2_loc],'time_step':i}
    return False


def detect_collisions(paths):
   
    collisions = []
    for i in range(0,len(paths)-1):
        for j in range(0,len(paths)):
            if i != j:
            
                collide = detect_collision(paths[i], paths[j])
                if collide:
                    collisions.append({'a1':i, 'a2':j, 'loc': collide['loc'], 'time_step': collide['time_step']})

    return collisions



def standard_splitting(collision):
    
    constraints = []

    if len(collision['loc']) == 1: #this is a vertex constraint
        constraints.append({'agent':collision['a1'], 'loc':collision['loc'], 'time_step':collision['time_step']})
        constraints.append({'agent':collision['a2'], 'loc':collision['loc'], 'time_step':collision['time_step']})
    else: #edge constraint
        constraints.append({'agent':collision['a2'], 'loc':collision['loc'], 'time_step':collision['time_step']})
        constraints.append({'agent':collision['a1'], 'loc':collision['loc'][::-1], 'time_step':collision['time_step']})

    return constraints


def disjoint_splitting(collision):
    
    constraints = []
    agentChoice = ['a1','a2']
    agent = random.choice(agentChoice)
   

    if len(collision['loc']) == 1: #vertex constraint
        constraints.append({'positive':True,'agent':collision[agent], 'loc':collision['loc'], 'time_step':collision['time_step']})
        constraints.append({'positive':False,'agent':collision[agent], 'loc':collision['loc'], 'time_step':collision['time_step']})
    
    else: #edge constraint (taken directly from standard splitting)
        constraints.append({'positive':False,'agent':collision['a2'], 'loc':collision['loc'], 'time_step':collision['time_step']})
        constraints.append({'positive':False,'agent':collision['a1'], 'loc':collision['loc'][::-1], 'time_step':collision['time_step']})

        # This is supposed to be what the hint tells us to do, fails for some test cases
        # if agent == 'a1': 
        #     constraints.append({'positive':True,'agent':collision[agent],'loc':collision['loc'][::-1],'time_step':collision['time_step']})
        #     constraints.append({'positive':False,'agent':collision[agent],'loc':collision['loc'][::-1],'time_step':collision['time_step']})
        # else:
        #     constraints.append({'positive':True,'agent':collision[agent], 'loc':collision['loc'], 'time_step':collision['time_step']})
        #     constraints.append({'positive':False,'agent':collision[agent], 'loc':collision['loc'], 'time_step':collision['time_step']})

    return constraints

def paths_violate_constraint(constraint, paths): #copied from paths_violate_constraint.py
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['time_step'])
        prev = get_location(paths[i], constraint['time_step'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        #print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        #print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        #print(root['collisions'])

        # Task 3.2: Testing
        #for collision in root['collisions']:
            #print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        
        while len(self.open_list)>0:
            next_node = self.pop_node()
            if len(next_node['collisions']) == 0:
                self.print_results(next_node)
                return next_node['paths']
            if disjoint:
                constraints = disjoint_splitting(next_node['collisions'][0])
            else:
                constraints = standard_splitting(next_node['collisions'][0])
            
            for constraint in constraints:
                #print(constraint)
                child = {'cost': 0,
                        'constraints': [],
                        'paths': [],
                        'collisions': []}            
                child['constraints'] = next_node['constraints'].copy()
                child['constraints'].append(constraint)
                child['paths'] = next_node['paths'].copy()
                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, child['constraints'])
                if path:
                    valid_flag = True
                    child['paths'][agent] = path    
                    if disjoint: #disjoint splitting
                        if constraint['positive'] == True:
                            violating_agents = paths_violate_constraint(constraint, child['paths'])
                            #need to append negative constraints for all agents who violate the positive constraint of current agent
                            for violating_agent in violating_agents:
                                child['constraints'].append({
                                    'positive': False,
                                    'agent':violating_agent,
                                    'loc':constraint['loc'],
                                    'time_step':constraint['time_step']
                                })
                                new_path = a_star(self.my_map, self.starts[violating_agent],self.goals[violating_agent], self.heuristics[violating_agent], violating_agent, child['constraints'])
                                if new_path:
                                    child['paths'][violating_agent] = new_path
                                else:
                                    valid_flag = False
                                    break
                                    
                   
                    if valid_flag:
                        child['collisions'] = detect_collisions(child['paths'])
                        child['cost'] = get_sum_of_cost(child['paths'])
                        self.push_node(child)

                    
                    
                    #return root['paths']           

        raise BaseException("No Solution")
        


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
