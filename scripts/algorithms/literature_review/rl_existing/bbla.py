#!/usr/bin/env python3

'''
BBLA
'''

import rospkg
import numpy as np
import rospy
import networkx as nx
from mrpp_sumo.srv import NextTaskBot, NextTaskBotResponse, AlgoReady, AlgoReadyResponse
from mrpp_sumo.msg import AtNode
import random as rn

class BBLA:

    def __init__(self, g, dis, eps, num_bots):
        self.ready = False
        self.graph = g
        self.stamp = 0.
        self.dis = dis
        self.eps = eps

        self.nodes = list(self.graph.nodes())
        for node in self.graph.nodes():
            self.graph.nodes[node]['idleness'] = 0.

        
        #initialize q values
        self.q_values = {}
        for node in self.graph.nodes():
            self.q_values[node] = {}
            for prev in self.graph.predecessors(node):
                self.q_values[node][prev] = {}
                for large in self.graph.successors(node):
                    self.q_values[node][prev][large] = {}
                    for small in self.graph.successors(node):
                        self.q_values[node][prev][large][small] = {}
                        for act in self.graph.successors(node):
                            self.q_values[node][prev][large][small][act] = {'val': 0., 'count': 0}

        #initialize robot last state action
        self.bot_last = {'bot_{}'.format(i): [] for i in range(num_bots)}

        rospy.Service('algo_ready', AlgoReady, self.callback_ready)
        self.ready = True

    def callback_idle(self, data):
        if self.stamp < data.stamp:
            dev = data.stamp - self.stamp
            self.stamp = data.stamp
            for node in self.nodes:
                self.graph.nodes[node]['idleness'] += dev

            for i, n in enumerate(data.robot_id):
                if self.bot_last[n] != []:
                    d = self.bot_last[n]
                    lr = 1/(2 + self.q_values[d[0]][d[1]][d[2]][d[3]][d[4]]['count']/15.)
                    node = data.node_id[i]
                    R = self.graph.nodes[node]['idleness']
                    prev = d[0]

                    max_idl = -np.inf
                    max_node = None
                    min_idl = np.inf
                    min_node = None

                    for nex in self.graph.successors(node):
                        if self.graph.nodes[nex]['idleness'] > max_idl:
                            if not (nex in data.node_id and max_idl > 0.):
                                max_idl = self.graph.nodes[nex]['idleness']
                                max_node = nex
                        
                        if self.graph.nodes[nex]['idleness'] < min_idl or nex in data.node_id:
                            min_idl = self.graph.nodes[nex]['idleness']
                            min_node = nex
                        
                            if nex in data.node_id:
                                min_idl = 0.

                    
                    qs = self.q_values[node][prev][max_node][min_node]
                    val = -np.inf
                    for act in qs.keys():
                        if qs[act]['val'] > val:
                            val = qs[act]['val']

                    self.q_values[d[0]][d[1]][d[2]][d[3]][d[4]]['val'] += lr * (self.dis ** self.stamp * R + self.dis * val - self.q_values[d[0]][d[1]][d[2]][d[3]][d[4]]['val'])

                self.graph.nodes[node]['idleness'] = 0.

    
    def callback_next_task(self, req):
        node = req.node_done
        t = req.stamp
        bot = req.name

        if self.bot_last[bot] != []:
            prev = self.bot_last[bot][0]
        else:
            prev = rn.sample(self.graph.predecessors(node), 1)[0]

        max_idl = -np.inf
        max_node = None
        min_idl = np.inf
        min_node = None

        for nex in self.graph.successors(node):
            if self.graph.nodes[nex]['idleness'] > max_idl:
                max_idl = self.graph.nodes[nex]['idleness']
                max_node = nex
            
            if self.graph.nodes[nex]['idleness'] < min_idl:
                min_idl = self.graph.nodes[nex]['idleness']
                min_node = nex
            

        prob = rn.random()
        if prob < self.eps:
            nex = rn.sample(self.graph.successors(node), 1)[0]

        else:
            nex = None
            qs = self.q_values[node][prev][max_node][min_node]
            val = -np.inf
            for act in qs.keys():
                if qs[act]['val'] > val:
                    val = qs[act]['val']
                    nex = act
            self.q_values[node][prev][max_node][min_node][nex]['count'] += 1

        self.bot_last[bot] = [node, prev, max_node, min_node, nex]
        next_walk = [node, nex]
        next_departs = [t]
        return NextTaskBotResponse(next_departs, next_walk)

    def callback_ready(self, req):
        algo_name = req.algo
        if algo_name == 'bbla' and self.ready:
            return AlgoReadyResponse(True)
        else:
            return AlgoReadyResponse(False)



if __name__ == '__main__':
    rospy.init_node('bbla', anonymous= True)
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    done = False
    graph_name = rospy.get_param('/graph')
    dis = float(rospy.get_param('/discount_factor'))
    eps = float(rospy.get_param('/exploration_rate'))
    num_bots = int(rospy.get_param('/init_bots'))

    g = nx.read_graphml(dirname + '/graph_ml/' + graph_name + '.graphml')

    s = BBLA(g, dis, eps, num_bots)

    rospy.Subscriber('at_node', AtNode, s.callback_idle)
    rospy.Service('bot_next_task', NextTaskBot, s.callback_next_task)

    done = False
    while not done:
        done = rospy.get_param('/done')