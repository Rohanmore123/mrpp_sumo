#!/usr/bin/env python3

'''
TPBP UTIL1

ROS Parameters -
1. graph
2. priority_nodes
3. time_periods
4. coefficients
5. num_dummy_nodes
6. reshuffle_time
7. random_string  (folder name)
'''
from platform import node
import rospy
import rospkg
import networkx as nx
import os
import numpy 

from mrpp_sumo.srv import NextTaskBot, NextTaskBotResponse
from mrpp_sumo.srv import AlgoReady, AlgoReadyResponse
from mrpp_sumo.msg import AtNode
import time
import random as rn
import numpy as np


def add_vertex_trail(graph, path, len_path, vertex, depth):
    cur = path[-1]
    len_rem = nx.dijkstra_path_length(graph, cur, vertex, weight = 'length')
    if len(path)-1 > depth:
        return False
    if not cur in path[:-1]:
        return True
    for i in range(len(path[:-2])):
        if path[i] == cur and path[i + 1] == vertex:
            return False
    return True

def compute_valid_trails(g,graph, source, len_max, depth, folder):

    with open(folder + '/depth_trails_{}_{}_{}.in'.format(str(g), str(source), str(depth)), 'w') as f:
        with open(folder + '/vp_temp_{}.in'.format(0), 'w') as f1:
            f1.write(str(source) + ' ' + str(0) + '\n')
        count = 1  #no of walks in vp temp
        steps = 0   # iterations on vp temp
        while count != 0:
            count = 0
            with open(folder + '/vp_temp_{}.in'.format(steps), 'r') as f0:
                with open(folder + '/vp_temp_{}.in'.format(steps + 1), 'w') as f1:
                    for line in f0:
                        line1 = line.split('\n')
                        line_temp = line1[0]
                        line1 = line_temp.split(' ')
                        path = list(map(str, line1[:-1]))
                        len_path = float(line1[-1])
                        neigh = graph.neighbors(path[-1])

                        for v in neigh:
                            ## VELOCITY is set to 10.m/s
                            if add_vertex_trail(graph, path, len_path, v, depth):
                                temp = ' '.join(line1[:-1])
                                temp = temp + ' ' + str(v)
                                count += 1
                                temp += ' ' + str(graph[path[-1]][v]['length'] + len_path)
                                f1.write(temp + '\n')
            steps += 1
            if steps >=depth:
                with open(folder + '/vp_temp_{}.in'.format(steps), 'r') as f0:
                    for line in f0:
                        line1 = line.split('\n')
                        line_temp = line1[0]
                        line1 = line_temp.split(' ')
                        path = ' '.join(line1[:-1])
                        for n in graph.nodes:
                            new_path=path
                            if n not in path:
                                add_path = nx.dijkstra_path(graph, line1[-2], n, weight='length')
                                new_path += " " + ' '.join(add_path[1:])
                                f.write(new_path+'\n')
                        
                break
            os.remove(folder + '/vp_temp_{}.in'.format(steps-1))
    os.remove(folder + '/vp_temp_{}.in'.format(steps))


def all_valid_trails(graph, node_set, len_max, depth, folder,g):
    for i in range(len(node_set)):
        compute_valid_trails(g, graph, node_set[i], len_max[i], depth, folder)


class TPBP:

    def __init__(self, graph, priority_nodes, time_periods, coefficients, depth, path_to_folder,graph_name):
        '''Initializes class TPBP with required ros paramenters  i.e time periods,dummy nodes reshuffle time,etc '''
        #velocity
        self.v_max=10
        self.robots = {}
        self.ready = False
        rospy.Service('algo_ready', AlgoReady, self.callback_ready)
        self.graph = graph          #assigning graph to self 
        self.priority_nodes = priority_nodes     
        self.time_periods = time_periods
        self.coefficients = coefficients
        self.depth = depth
        self.graph_name = graph_name
        self.offline_folder = path_to_folder   #offline folder path to store valid walks
        for node in self.graph.nodes():
            self.graph.nodes[node]['idleness'] = 0.     #adding a idleness parameter to nodes
            self.graph.nodes[node]['future_visits'] = {}
        self.stamp = 0.         #initializing time stamp to 0
        self.nodes = list(self.graph.nodes())
        self.visit_counter = np.zeros(len(self.priority_nodes))
        
        self.N = len(self.nodes)
        self.assigned = []
        #self.non_priority_assigned = []
        for _ in self.priority_nodes:
            self.assigned.append(False)

        self.tpbp_offline()

    def tpbp_offline(self):
        if not os.path.isdir(self.offline_folder):
            '''creates a offline folder is not created already'''
            os.mkdir(self.offline_folder)
        s = len(self.priority_nodes)
        temp = self.time_periods.copy()
        for _ in range(s):
            ''' assigning time periods to non priority nodes'''
            temp.append(self.time_periods[0])
        all_valid_trails(self.graph, self.priority_nodes, temp, self.depth, self.offline_folder, self.graph_name)
        time.sleep(1.)  #halts the exceution of code for 1 sec
        self.ready = True


    def utility(self,walk):
        reward = 0
        n=[0]
        node_val = np.zeros(self.N)
        future_visit_final = {}
        for i, w in enumerate(self.nodes):
            node_val[i] = self.graph.nodes[w]['idleness']
            if len(self.graph.nodes[w]['future_visits'].values()) > 0:
                future_visit_final[w] = max(self.graph.nodes[w]['future_visits'].values())
            else:
                future_visit_final[w] = -1. * node_val[i]

        for i,j,z in list(zip(walk[:-1],walk[1:],list(range(len(walk)-1)))):
            n.append(n[z]+self.graph[i][j]['length']/self.v_max)
        for t,i in enumerate(walk):
            if future_visit_final[i] > n[t]:
                reward += 0

            else:
                reward += n[t] - future_visit_final[i]
                if i in self.priority_nodes:
                    reward += (self.coefficients[1])*(n[t] - future_visit_final[i])
                future_visit_final[i]=n[t]
        return reward  


    def callback_idle(self, data):
        #Update Idleness
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

    def callback_next_task(self, req):
        t = req.stamp
        node = req.node_done

        if node in self.priority_nodes:
            self.assigned[self.priority_nodes.index(node)] = False

        print (node, self.priority_nodes, self.visit_counter)

        best_reward = -np.inf
        next_walk = []

        self.graph.nodes[node]['idleness'] = 0.

        #taking least visited node
        list_min = np.where(self.visit_counter==np.amin(self.visit_counter))       #list min contains index values of least visited it is list in list
        
        if len(list_min[0]) == 1:
            j=list_min[0][0]
            if not self.assigned[j]:
                valid_trails = '/depth_trails_{}_{}_{}.in'.format(self.graph_name, node, str(self.depth))
                with open(self.offline_folder + valid_trails, 'r') as f:
                    count = 0
                    for line in f:
                        count += 1
                        line1 = line.split('\n')
                        line2 = line1[0].split(' ')
                        line3 = nx.dijkstra_path(self.graph, line2[-1], self.priority_nodes[j], weight='length')
                        if line2[-1] not in self.priority_nodes:
                            line2.extend(line3[1:])
                        r = self.utility(line2)
                        if r > best_reward:
                            best_reward = r
                            next_walk = line2
        else:
            priority_idle = {}
            for n in list_min[0]:
                priority_idle[self.priority_nodes[n]] = self.graph.nodes[self.priority_nodes[n]]['idleness']

            P_idle_node = max(priority_idle,key=priority_idle.get)

            valid_trails = '/depth_trails_{}_{}_{}.in'.format(self.graph_name, node, str(self.depth))
            with open(self.offline_folder + valid_trails, 'r') as f:
                count = 0
                for line in f:
                    count += 1
                    line1 = line.split('\n')
                    line2 = line1[0].split(' ')
                    line3 = nx.dijkstra_path(self.graph, line2[-1], P_idle_node, weight='length')
                    if line2[-1] not in self.priority_nodes:
                        line2.extend(line3[1:])
                    r = self.utility(line2)
                    if r > best_reward:
                        best_reward = r
                        next_walk = line2
        self.visit_counter[self.priority_nodes.index(next_walk[-1])] += 1
        next_departs = [t] * (len(next_walk) - 1)
        
        return NextTaskBotResponse(next_departs, next_walk)


    def callback_ready(self, req):
        algo_name = req.algo
        if algo_name == 'through_modified_FHUM' and self.ready:
            return AlgoReadyResponse(True)
        else:
            return AlgoReadyResponse(False)

if __name__ == '__main__':
    rospy.init_node('through_modified_FHUM', anonymous = True)
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    done = False
    graph_name = rospy.get_param('/graph')
    g = nx.read_graphml(dirname + '/graph_ml/' + graph_name + '.graphml')
    algo_name=rospy.get_param('/algo_name')
    priority_nodes = rospy.get_param('/priority_nodes').split(' ')
    time_periods = list(map(float, rospy.get_param('/time_periods').split(' ')))
    coefficients = list(map(float, rospy.get_param('/coefficients').split(', ')))
    depth = rospy.get_param('/depth')
    folder = algo_name + graph_name
    path_to_folder = dirname + '/outputs/' + folder
    s = TPBP(g, priority_nodes, time_periods, coefficients, depth, path_to_folder,graph_name)

    rospy.Subscriber('at_node', AtNode, s.callback_idle)
    rospy.Service('bot_next_task', NextTaskBot, s.callback_next_task)
    done = False
    while not done:
        done = rospy.get_param('/done')
        rospy.sleep(0.1)