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

def add_vertex_trail(path, vertex, depth):
    '''
    Check whether the path's a trail
    '''
    cur = path[-1]
    if len(path) > depth:
        return False
    if not cur in path[:-1]:
        return True
    for i in range(len(path[:-2])):
        if path[i] == cur and path[i + 1] == vertex:
            return False
    return True

def compute_valid_trails(name_graph, graph, source, depth, folder, priority_nodes):

    '''
    returns files named as 'path_to_folder/graph_source_dest_depth.in'
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
                    for v in neigh:
                        ## VELOCITY is set to 10.m/s
                        if add_vertex_trail(path, v, depth):
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
            for n in list(graph.nodes()):
                path_1 = path.copy()
                # print(n, n not in path)
                if n not in path:
                    path_2 = nx.dijkstra_path(graph, path[-1], n, weight = 'length')
                    path_1.extend(path_2[1:])
                    for dest in priority_nodes:
                        with open(folder + '/depth_trails_{}_{}_{}_{}.in'.format(str(name_graph), str(source), str(dest), str(depth)), 'a+') as f:
                            path_3 = nx.dijkstra_path(graph, path_1[-1], dest, weight = 'length')
                            path_1.extend(path_3[1:])
                            new_path = ' '.join(path_1)               
                            f.write(new_path + '\n')

    for i in range(depth):        
        os.remove(folder + '/vp_temp_{}.in'.format(i))


def all_valid_trails(graph, node_set, depth, folder, graph_name):
    for i in range(len(node_set)):
        compute_valid_trails(graph_name, graph, node_set[i], depth, folder, node_set)


class PPABasic:

    '''
    Base class
    '''

    def __init__(self, graph, priority_nodes, l_prior, depth, path_to_folder, graph_name):
        '''Initializes class with required ros paramenters'''
        
        self.ready = False
        rospy.Service('algo_ready', AlgoReady, self.callback_ready)
        self.graph = graph          #assigning graph to self 
        self.priority_nodes = priority_nodes     
        self.l_prior = l_prior
        self.depth = depth
        self.graph_name = graph_name
        self.offline_folder = path_to_folder   #offline folder path to store valid walks
        for node in self.graph.nodes():
            self.graph.nodes[node]['idleness'] = 0.     #adding a idleness parameter to nodes
        self.stamp = 0.         #initializing time stamp to 0
      
        self.nodes = list(self.graph.nodes())
        self.visit_counter = np.zeros(len(self.priority_nodes))
        self.assigned = []
        for _ in self.priority_nodes:
            self.assigned.append(False)

        self.tpbp_offline()

    def tpbp_offline(self):
        '''
        Create offline directory
        '''
        if not os.path.isdir(self.offline_folder):
            '''creates a offline folder is not created already'''
            os.mkdir(self.offline_folder)
        #n = len(list(self.graph.nodes()))

        all_valid_trails(self.graph, self.priority_nodes, self.depth, self.offline_folder, self.graph_name)
        time.sleep(1.)  #halts the exceution of code for 1 sec
        self.ready = True

    def utility(self, path):
        '''
        Basic Utility function (base)
        '''
        nodes = list(set(path))
        reward = 0.
        for i in nodes:
            reward += self.graph.nodes[i]['idleness']
            if i in self.priority_nodes:
                reward += self.l_prior * self.graph.nodes[i]['idleness']
        return reward  


    def callback_idle(self, data):
        '''
        Update Idleness
        '''
        #Update Idleness
        if self.stamp < data.stamp:
            dev = data.stamp - self.stamp
            self.stamp = data.stamp
            for i in self.graph.nodes():
                self.graph.nodes[i]['idleness'] += dev
            for i, n in enumerate(data.node_id):
                self.graph.nodes[n]['idleness'] = 0.

    def callback_next_task(self, req):
        '''
        Assign next task to the robot
        '''
        t = req.stamp
        node = req.node_done

        if node in self.priority_nodes:
            self.assigned[self.priority_nodes.index(node)] = False

        print (node, self.priority_nodes, self.visit_counter)

        best_reward = -np.inf
        next_walk = []

        self.graph.nodes[node]['idleness'] = 0.

        #taking least visited node
        list_min = np.where(self.visit_counter == np.amin(self.visit_counter))       #list min contains index values of least visited it is list in list
        if len(list_min[0]) == 1:
            j = list_min[0][0]
        else:
            priority_idle = {}
            for n in list_min[0]:
                priority_idle[n] = self.graph.nodes[self.priority_nodes[n]]['idleness']
            j = max(priority_idle, key = priority_idle.get)
        valid_trails = '/depth_trails_{}_{}_{}_{}.in'.format(self.graph_name, node, self.priority_nodes[j], str(self.depth))
        with open(self.offline_folder + valid_trails, 'r') as f:
            count = 0
            for line in f:
                count = 0
                for line in f:
                    count += 1
                    path = line.split('\n')[0].split(' ')

                    r = self.utility(path)
                    if r > best_reward:
                        best_reward = r
                        next_walk = path

        self.visit_counter[self.priority_nodes.index(next_walk[-1])] += 1
        next_departs = [t] * (len(next_walk) - 1)
        return NextTaskBotResponse(next_departs, next_walk)


    def callback_ready(self, req):
        algo_name = req.algo
        if algo_name == 'ppa_basic' and self.ready:
            return AlgoReadyResponse(True)
        else:
            return AlgoReadyResponse(False)

if __name__ == '__main__':
    rospy.init_node('ppa_basic', anonymous = True)
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
    s = PPABasic(g, priority_nodes, l_prior, depth, path_to_folder, graph_name)

    rospy.Subscriber('at_node', AtNode, s.callback_idle)
    rospy.Service('bot_next_task', NextTaskBot, s.callback_next_task)
    done = False
    while not done:
        done = rospy.get_param('/done')
        rospy.sleep(0.1)