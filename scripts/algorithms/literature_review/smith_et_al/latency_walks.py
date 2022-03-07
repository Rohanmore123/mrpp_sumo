#!/usr/bin/env python3

'''
Latency Walks Sumo Implementation
'''

import rospkg
import numpy as np
import persistent_monitoring_multi_agent as pmm
import rospy
import networkx as nx
from mrpp_sumo.srv import NextTaskBot, NextTaskBotResponse, AlgoReady, AlgoReadyResponse
# from mrpp_sumo.msg import AtNode

class LW:

    '''
    Sumo Wrapper for Latency Walk algo with priority node setting
    '''

    def __init__(self, g, rho, num_robots, prior_nodes):
        '''
        Initialize
        '''
        self.ready = False
        self.graph = g
        self.num_robots = num_robots
        self.rho = rho
        self.priority_nodes = prior_nodes
        self.node_weights = {n: 1 for n in list(g.nodes())}
        self.non_priority_nodes = [n for n in list(g.nodes()) if n not in prior_nodes]
        for n in self.non_priority_nodes:
            self.node_weights[n] /= self.rho
        self.robot_walks = pmm.latency_walks(self.graph, self.node_weights, self.num_robots)
        rospy.Service('algo_ready', AlgoReady, self.callback_ready)
        self.robot_loc = [-1 for _ in range(num_robots)]
        self.ready = True

    def callback_ready(self, req):
        '''
        Ready Callback
        '''
        algo_name = req.algo
        if algo_name == 'latency_walks' and self.ready:
            return AlgoReadyResponse(True)
        else:
            return AlgoReadyResponse(False)

    def callback_next_task(self, req):
        '''
        Next Task Callback
        '''
        node = req.node_done
        t = req.stamp
        bot = req.name

        bot_id = int(bot.split('_')[-1])
        self.robot_loc[bot_id] += 1 
        
        self.robot_loc[bot_id] %= len(self.robot_walks[bot_id])

        next_walk = [node, self.robot_walks[bot_id][int(self.robot_loc[bot_id])]]
        next_departs = [t] * (len(next_walk) - 1)
        return NextTaskBotResponse(next_departs, next_walk)

    
if __name__ == '__main__':
    rospy.init_node('lw', anonymous = True)
    dir_name = rospkg.RosPack().get_path('mrpp_sumo')
    done = False
    graph_name = rospy.get_param('/graph')
    g = nx.read_graphml(dir_name + '/graph_ml/' + graph_name + '.graphml')
    num_robots = int(rospy.get_param('/init_bots'))
    rho = float(rospy.get_param('/parameters'))
    prior_nodes = rospy.get_param('/priority_nodes').split(' ')
    s = LW(g, rho, num_robots, prior_nodes)

    rospy.Service('bot_next_task', NextTaskBot, s.callback_next_task)

    done = False
    while not done:
        done = rospy.get_param('/done')