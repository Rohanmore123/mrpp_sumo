#!/usr/bin/env python3

'''
Estimated idleness is the mean of true idlenesses observed by every agent based on self visit history 
Decision making is based on max q val
'''

import rospkg
import numpy as np
import rospy
import networkx as nx
from mrpp_sumo.srv import NextTaskBot, NextTaskBotResponse, AlgoReady, AlgoReadyResponse
from mrpp_sumo.msg import AtNode
import random as rn

class LEDGE:

    def __init__(self, g, num_bots, error_file):
        self.ready = False
        self.graph = g
        self.stamp = 0.
        self.num_bots = num_bots
        self.vel = 10. #MAX VELOCITY SET FOR SIMULATIONS
        
        self.nodes = list(self.graph.nodes())
        self.est_idle = {}
        self.num_traversals = {}
        self.current_node = {}
        self.q_val = {}
        for i in range(num_bots):
            self.est_idle['bot_{}'.format(i)] = {}
            self.num_traversals['bot_{}'.format(i)] = {}
            self.current_node['bot_{}'.format(i)] = None
            self.q_val['bot_{}'.format(i)] = {}
            for n in self.nodes:
                self.est_idle['bot_{}'.format(i)][n] = {}
                self.num_traversals['bot_{}'.format(i)][n] = {}
                self.q_val['bot_{}'.format(i)][n] = {}
                for m in list(self.graph.successors(n)):
                    self.est_idle['bot_{}'.format(i)][n][m] = self.graph[n][m]['length']/self.vel
                    self.num_traversals['bot_{}'.format(i)][n][m] = 0
                    self.q_val['bot_{}'.format(i)][n][m] = 0

        self.error_file = open(error_file, 'a+')

        self.idleness = {n: 0 for n in self.nodes}        
        
        rospy.Service('algo_ready', AlgoReady, self.callback_ready)
        self.ready = True

    def callback_idle(self, data):
        if self.stamp < data.stamp:
            dev = data.stamp - self.stamp
            self.stamp = data.stamp
            # for n in range(num_bots):
            #     for i in self.nodes:
            #         self.est_idle['bot_{}'.format(n)][i] += dev

            for n in self.nodes:
                self.idleness[n] += dev            
                
            for i, n in enumerate(data.robot_id):
                if not self.current_node[n] is None and (self.current_node[n], data.node_id[i]) in self.graph.edges():
                    self.num_traversals[n][self.current_node[n]][data.node_id[i]] += 1 
                    count = self.num_traversals[n][self.current_node[n]][data.node_id[i]]
                    est = self.est_idle[n][self.current_node[n]][data.node_id[i]]
                    tr = self.idleness[data.node_id[i]]
                    if abs(tr - est) > 1:
                        self.est_idle[n][self.current_node[n]][data.node_id[i]] = ((count - 1) * self.est_idle[n][self.current_node[n]][data.node_id[i]] + self.idleness[data.node_id[i]]) / count
                        self.q_val[n][self.current_node[n]][data.node_id[i]] += np.sign(tr - est)

                    self.error_file.write('{},{},{},{},{}\n'.format(self.stamp, data.node_id[i], n, self.est_idle[n][self.current_node[n]][data.node_id[i]], self.idleness[data.node_id[i]]))
                self.idleness[data.node_id[i]] = 0.
                
    
    def callback_next_task(self, req):
        node = req.node_done
        t = req.stamp
        bot = req.name

        neigh = list(self.graph.successors(node))
        idles = []
        for n in neigh:
            idles.append(np.exp(self.q_val[bot][node][n]))

        tots = sum(idles)
        idles = [i/tots for i in idles]

        max_id = 0
        if len(neigh) > 1:
            max_ids = list(np.where(idles == np.amax(idles))[0])
            max_id = rn.sample(max_ids, 1)[0]
        next_walk = [node, neigh[max_id]]
        next_departs = [t]
        self.current_node[bot] = node
        return NextTaskBotResponse(next_departs, next_walk)

    def callback_ready(self, req):
        algo_name = req.algo
        if algo_name == 'ledge_4' and self.ready:
            return AlgoReadyResponse(True)
        else:
            return AlgoReadyResponse(False)

if __name__ == '__main__':
    rospy.init_node('ledge_4', anonymous= True)
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    done = False
    graph_name = rospy.get_param('/graph')
    num_bots = int(rospy.get_param('/init_bots'))
    g = nx.read_graphml(dirname + '/graph_ml/' + graph_name + '.graphml')
    exp_name = rospy.get_param('/random_string')
    error_file = dirname + '/outputs/{}_error_file.csv'.format(exp_name)
    s = LEDGE(g, num_bots, error_file)

    rospy.Subscriber('at_node', AtNode, s.callback_idle)
    rospy.Service('bot_next_task', NextTaskBot, s.callback_next_task)

    done = False
    while not done:
        done = rospy.get_param('/done')

    s.error_file.close()