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

    with open(folder + '/vp_temp_{}.in'.format(0), 'w') as f1:
        f1.write(str(source) + '\n')
    count = 1  #no of walks in vp temp
    steps = 0   # iterations on vp temp
    
    print('1')
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
        print(steps)

    with open(folder + '/vp_temp_{}.in'.format(depth), 'r') as f0:
        for line in f0:
            line1 = line.split('\n')
            path = line1[0].split(' ')
            print(path)
            for n in list(graph.nodes()):
                path_1 = path.copy()
                print(n, n not in path)
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


def all_valid_trails(graph, node_set, len_max, depth, folder, g):
    for i in range(len(node_set)):
        compute_valid_trails(g, graph, node_set[i], len_max[i], depth, folder)

class PPAFHUM(g, priority_nodes, l_prior, depth, path_to_folder, graph_name):

    def __init__(self, graph, priority_nodes, l_prior, depth, path_to_folder, graph_name):
        '''Initializes class with required ros parameters'''

        self.robots = {}
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
            self.graph.nodes[node]['future_visits'] = {}
        self.stamp = 0.         #initializing time stamp to 0
        self.nodes = list(self.graph.nodes())

        # self.visit_counter = np.zeros(len(self.priority_nodes))
        
        self.N = len(self.nodes)
        self.tpbp_offline()

    def tpbp_offline(self):
        if not os.path.isdir(self.offline_folder):
            '''creates a offline folder is not created already'''
            os.mkdir(self.offline_folder)

        all_valid_trails(self.graph, self.priority_nodes, temp, self.depth, self.offline_folder, self.graph_name)
        time.sleep(1.)  #halts the exceution of code for 1 sec
        self.ready = True
    

if __name__ == '__main__':
    rospy.init_node('ppa_fhum', anonymous = True)
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