#!/usr/bin/env python3

'''
Go to the neighbouring node
Share intent - account for intent via idleness

Agents' idleness estimate is equal to true idleness based on edge weight and expected visit instance of other bots

'''


import rospy
import rospkg
import networkx as nx
import numpy as np
import random as rn

from mrpp_sumo.srv import NextTaskBot, NextTaskBotResponse
from mrpp_sumo.srv import AlgoReady, AlgoReadyResponse
from mrpp_sumo.msg import AtNode 

class ROLL:
    def __init__(self, graph, num_bots):
        self.ready = False
        self.graph = graph
        self.stamp = 0.
        self.vel = 10.  #VELOCITY TO BE SET TO 10 DURING SIMULATIONS
        self.ready = True
        self.epsilon = 0.9
        self.gamma = 0.9
        self.alpha = 0.1
        self.last_node = {}
        for i in range(num_bots):
            self.last_node['bot_{}'.format(i)] = None
        self.nodes = list(self.graph.nodes())
        self.intent = {}
        self.idleness = {}
        self.q_vals = {}
        for n in self.nodes:
            self.intent[n] = {}
            self.idleness[n] = 0.
            self.q_vals[n] = {}
            for m in self.graph.successors(n):
                self.q_vals[n][m] = 0 

    def callback_idle(self, data):
        #Update idleness of the nodes in the graph
        if self.stamp < data.stamp:
            dev = data.stamp - self.stamp
            self.stamp = data.stamp
            for n in self.nodes:
                self.idleness[n] += dev
                
            for i, n in enumerate(data.robot_id):
                cur_node = data.node_id[i]
                neigh = list(self.graph.successors(cur_node))
                nex_vals = [self.q_vals[cur_node][next_node] for next_node in neigh]
                max_ids = list(np.where(nex_vals == np.amax(nex_vals))[0])
                max_id = int(rn.sample(max_ids, 1)[0])
                max_node = neigh[max_id]

                if (not self.last_node[n] is None) and (self.last_node[n] != cur_node):
                    # print('here')
                    self.q_vals[self.last_node[n]][cur_node] += self.alpha * (self.idleness[cur_node] + self.gamma * (self.q_vals[cur_node][max_node] - self.q_vals[self.last_node[n]][cur_node]))
 
                self.idleness[data.node_id[i]] = 0.
                            

    def callback_next_task(self, req):
        node = req.node_done
        t = req.stamp
        bot = req.name
        self.last_node[bot] = node
        self.idleness[node] = 0
        if bot in self.intent[node].keys():
            if self.intent[node][bot] < t:
                self.intent[node].pop(bot)

        neigh = list(self.graph.successors(node))
        e = rn.random()
        if e > self.epsilon:
            next_node = rn.sample(neigh, 1)[0]
        else:
            idles = []
            for n in neigh:
                idle = self.idleness[n] + self.graph[node][n]['length']/self.vel
                fut = 0
                for i in self.intent[n].values():
                    if i > fut:
                        fut = i
                idle -= fut
                idle += self.gamma * max(list(self.q_vals[n].values()))
                idles.append(idle)

            max_id = 0
            if len(neigh) > 1:
                max_ids = list(np.where(idles == np.amax(idles))[0])
                max_id = rn.sample(max_ids, 1)[0]
            next_node = neigh[max_id]

        next_walk = [node, next_node]
        next_departs = [t]

        self.intent[next_node][bot] = t + self.graph[node][next_node]['length']/self.vel


        return NextTaskBotResponse(next_departs, next_walk)

    def callback_ready(self, req):
        algo_name = req.algo
        if algo_name == 'alg_3' and self.ready:
            return AlgoReadyResponse(True)
        else:
            return AlgoReadyResponse(False)

if __name__ == '__main__':
    rospy.init_node('alg_3', anonymous = True)
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    done = False
    graph_name = rospy.get_param('/graph')
    num_bots = rospy.get_param('/init_bots')
    g = nx.read_graphml(dirname + '/graph_ml/' + graph_name + '.graphml')
    
    s = ROLL(g, num_bots)
    rospy.Subscriber('at_node', AtNode, s.callback_idle)
    rospy.Service('bot_next_task', NextTaskBot, s.callback_next_task)
    rospy.Service('algo_ready', AlgoReady, s.callback_ready)
    while not done:
        done = rospy.get_param('/done')