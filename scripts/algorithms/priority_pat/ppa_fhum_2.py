#!/usr/bin/env python3

import rospy
import rospkg
import networkx as nx
import os

from mrpp_sumo.srv import NextTaskBot, NextTaskBotResponse
from mrpp_sumo.srv import AlgoReady, AlgoReadyResponse
from mrpp_sumo.msg import AtNode
import time
import random as rn
import numpy as np

def add_vertex_trail(path, vertex):
    '''
    Check whether the path's a trail
    '''
    cur = path[-1]

    if not cur in path[:-1]:
        return True
    for i in range(len(path[:-2])):
        if path[i] == cur and path[i + 1] == vertex:
            return False
    return True

def compute_valid_trails(name_graph, graph, source, depth, folder, priority_nodes):

    '''
    returns files named as 'path_to_folder/graph_source_dest_depth.in' - modified to not contain any priority nodes in the interior of path
    '''

    for dest in priority_nodes:
        with open(folder + '/depth_trails_{}_{}_{}_{}.in'.format(str(name_graph), str(source), str(dest), str(depth)), 'w') as f:
            pass


    with open(folder + '/vp_temp_{}.in'.format(0), 'w') as f1:
        f1.write(str(source) + '\n')
    count = 1  #no of walks in vp temp
    steps = 0   # iterations on vp temp
    
    # print('1')
    while count != 0 and steps < (depth):
        count = 0
        with open(folder + '/vp_temp_{}.in'.format(steps), 'r') as f0:
            with open(folder + '/vp_temp_{}.in'.format(steps + 1), 'w') as f1:
                for line in f0:
                    line1 = line.split('\n')
                    path = line1[0].split(' ')
                    neigh = graph.neighbors(path[-1])
                    # print(line)
                    prior = False
                    for n in priority_nodes:
                        if len(path) >= 2 and n in path[1:]:
                            prior = True
                    if prior:
                        temp = ' '.join(path) + '\n'
                        count += 1
                        f1.write(temp)
                    else:
                        for v in neigh:
                            ## VELOCITY is set to 10.m/s
                            if add_vertex_trail(path, v):
                                temp = ' '.join(path)
                                temp = temp + ' ' + str(v) + '\n'
                                count += 1
                                f1.write(temp)
                        
        steps += 1
        # print(steps)

    with open(folder + '/vp_temp_{}.in'.format(depth), 'r') as f0:
        for line in f0:
            line1 = line.split('\n')
            path = line1[0].split(' ')
            # print(path)
            if path[-1] in priority_nodes:
                with open(folder + '/depth_trails_{}_{}_{}_{}.in'.format(str(name_graph), str(source), str(path[-1]), str(depth)), 'a+') as f:
                    new_path = ' '.join(path)
                    f.write(new_path + '\n')
            else:        
                for n in list(graph.nodes()):
                    path_1 = path.copy()
                    # print(n, n not in path)
                    if n not in path:
                        path_2 = nx.dijkstra_path(graph, path[-1], n, weight = 'length')
                        path_1.extend(path_2[1:])
                        if path_1[-1] in priority_nodes:
                            with open(folder + '/depth_trails_{}_{}_{}_{}.in'.format(str(name_graph), str(source), str(path_1[-1]), str(depth)), 'a+') as f:
                                new_path = ' '.join(path)
                                f.write(new_path + '\n')
                        else:
                            prior1 = False
                            for m in priority_nodes:
                                if m in path_1[1:]:
                                    prior1 = True
                            if not prior1:
                                for dest in priority_nodes:
                                    path_3 = nx.dijkstra_path(graph, path_1[-1], dest, weight = 'length')
                                    path_1.extend(path_3[1:])
                                    prior2 = False
                                    for m in priority_nodes:
                                        if m in path_3[1:-1]:
                                            prior2 = True
                                    if not prior2:
                                        with open(folder + '/depth_trails_{}_{}_{}_{}.in'.format(str(name_graph), str(source), str(dest), str(depth)), 'a+') as f:
                                            new_path = ' '.join(path_1)               
                                            f.write(new_path + '\n')

    for i in range(depth):        
        os.remove(folder + '/vp_temp_{}.in'.format(i))


def all_valid_trails(graph, node_set, depth, folder, graph_name):
    for i in range(len(node_set)):
        compute_valid_trails(graph_name, graph, node_set[i], depth, folder, node_set)

class PPAFHUM:

    '''
    Second variation of priority patrolling algorithm
    '''
    def __init__(self, graph, priority_nodes, l_prior, depth, path_to_folder, graph_name):
        '''Initializes class with required ros parameters'''

        self.robots = {}
        self.ready = False
        self.v_max = 10 #Velocity set to 10m/s
        rospy.Service('algo_ready', AlgoReady, self.callback_ready)
        
        self.graph = graph          #assigning graph to self 
        self.priority_nodes = priority_nodes     
        self.l_prior = l_prior
        self.depth = depth
        self.graph_name = graph_name
        self.offline_folder = path_to_folder   #offline folder path to store valid walks
        for node in self.graph.nodes():
            self.graph.nodes[node]['idleness'] = 0.     #adding a idleness parameter to nodes
            self.graph.nodes[node]['future_visits'] = {}
        self.stamp = 0.         #initializing time stamp to 0
        self.nodes = list(self.graph.nodes())
        
        self.N = len(self.nodes)
        self.tpbp_offline()

    def tpbp_offline(self):
        '''
        Create offline directory
        '''
        if not os.path.isdir(self.offline_folder):
            # creates a offline folder is not created already
            os.mkdir(self.offline_folder)

        all_valid_trails(self.graph, self.priority_nodes, self.depth, self.offline_folder, self.graph_name)
        time.sleep(1.)  #halts the exceution of code for 1 sec
        self.ready = True

    def utility(self, path):
        '''
        Utility function (base)
        '''
        reward = 0
        n = [0]
        node_val = np.zeros(self.N)
        future_visit_final = {}
        for i, w in enumerate(self.nodes):
            node_val[i] = self.graph.nodes[w]['idleness']
            if len(self.graph.nodes[w]['future_visits'].values()) > 0:
                future_visit_final[w] = max(self.graph.nodes[w]['future_visits'].values())
            else:
                future_visit_final[w] = -1. * node_val[i]

        for i,j,z in list(zip(path[:-1], path[1:], list(range(len(path)-1)))):
            n.append(n[z] + self.graph[i][j]['length']/self.v_max)

        for t, i in enumerate(path):
            if future_visit_final[i] > n[t]:
                reward += 0

            else:
                reward += n[t] - future_visit_final[i]
                if i in self.priority_nodes:
                    reward += (self.l_prior) * (n[t] - future_visit_final[i])
                future_visit_final[i]=n[t]
        return reward  


    def callback_next_task(self, req):
        '''
        Assign next task to the robot
        '''

        t = req.stamp
        node = req.node_done

        best_reward = -np.inf
        next_walk = []

        self.graph.nodes[node]['idleness'] = 0.

        for dest in self.priority_nodes:
            
            valid_trails = '/depth_trails_{}_{}_{}_{}.in'.format(self.graph_name, node, dest, str(self.depth))
            if os.path.isfile(self.offline_folder + valid_trails):
                with open(self.offline_folder + valid_trails, 'r') as f:
                    count = 0
                    for line in f:
                        count += 1
                        path = line.split('\n')[0].split(' ')

                        r = self.utility(path)
                        if r > best_reward:
                            best_reward = r
                            next_walk = path
        
        next_departs = [t] * (len(next_walk) - 1)
        
        return NextTaskBotResponse(next_departs, next_walk)

    def callback_idle(self, data):
        '''
        Update idleness and future visits
        '''
        if self.stamp < data.stamp:
            dev = data.stamp - self.stamp
            self.stamp = data.stamp
            for i in self.graph.nodes():
                self.graph.nodes[i]['idleness'] += dev
                for j in self.graph.nodes[i]['future_visits'].keys():
                    self.graph.nodes[i]['future_visits'][j] -= dev

            for i, n in enumerate(data.node_id):
                self.graph.nodes[n]['idleness'] = 0.
                if data.robot_id[i] in self.graph.nodes[n]['future_visits'].keys():
                    if self.graph.nodes[n]['future_visits'][data.robot_id[i]] < 0.:
                        self.graph.nodes[n]['future_visits'].pop(data.robot_id[i])


    def callback_ready(self, req):
        '''
        Verify algorithm
        '''
        if req.algo == 'ppa_fhum_2' and self.ready:
            return AlgoReadyResponse(True)
        else:
            return AlgoReadyResponse(False)
    

if __name__ == '__main__':
    rospy.init_node('ppa_fhum_2', anonymous = True)
    dirname = rospkg.RosPack().get_path('mrpp_sumo')

    graph_name = rospy.get_param('/graph')
    g = nx.read_graphml(dirname + '/graph_ml/' + graph_name + '.graphml')
    algo_name = rospy.get_param('/algo_name')
    priority_nodes = rospy.get_param('/priority_nodes').split(' ')
    parameters = list(map(float, rospy.get_param('/parameters').split(' ')))
    l_prior = parameters[0]
    depth = int(parameters[1])
    folder = algo_name + '_' + graph_name
    path_to_folder = dirname + '/outputs/' + folder
    s = PPAFHUM(g, priority_nodes, l_prior, depth, path_to_folder, graph_name)

    rospy.Subscriber('at_node', AtNode, s.callback_idle)
    rospy.Service('bot_next_task', NextTaskBot, s.callback_next_task)
    done = False
    while not done:
        done = rospy.get_param('/done')
        rospy.sleep(0.1)