#!/usr/bin/env python3


'''
modification to print out value_coeff
CR_MRPP

ROS Params
'''
import rospy
import rospkg
import numpy as np
import networkx as nx
from mrpp_sumo.srv import NextTaskBot, NextTaskBotResponse
from mrpp_sumo.srv import AlgoReady, AlgoReadyResponse
from mrpp_sumo.msg import AtNode
import random as rn
import csv
import math

class AN_MRPP:

    def __init__(self, graph, num_bots):
        self.ready = False
        self.graph = graph
        self.stamp = 0.
        self.num_bots = num_bots
        self.nodes = list(self.graph.nodes())

        self.idle_expect = {}
        self.idle_true = {}
        self.average = {}
        self.value_coeff = {}
        self.current_node = {}
        self.old_node = {}

        for i in range(self.num_bots):
            self.idle_expect['bot_{}'.format(i)] = {}
            self.average['bot_{}'.format(i)] = {}
            self.value_coeff['bot_{}'.format(i)] = {}
            self.current_node['bot_{}'.format(i)] = 0.
            self.old_node['bot_{}'.format(i)] = 0.

            for n in self.nodes:
                self.idle_expect['bot_{}'.format(i)][n] = {}
                self.average['bot_{}'.format(i)][n] = 0.
                self.value_coeff['bot_{}'.format(i)][n] = {}

                for m in self.graph.successors(n):
                    self.idle_expect['bot_{}'.format(i)][n][m] = 0.
                    self.value_coeff['bot_{}'.format(i)][n][m] = 1./(len(list(self.graph.successors(n))))

        for n in self.nodes:
            self.idle_true[n] = 0.

        self.ready = True

    def callback_idle(self, data):
        if self.stamp < data.stamp:
            dev = data.stamp - self.stamp
            self.stamp = data.stamp

            for i in range(self.num_bots):
                for n in self.nodes:
                    for m in self.graph.successors(n):
                        self.idle_expect['bot_{}'.format(i)][n][m] += dev

            for i in self.nodes:
                self.idle_true[i] += dev

            for i in self.idle_expect.keys():
                sum = 0
                len = 0

                for j in self.idle_expect[i].keys():
                    for m in self.idle_expect[i][j].keys():
                        sum += self.idle_expect[i][j][m]
                        len += 1

                    self.average[i][j] = sum/len


            for i, n in enumerate(data.robot_id):
                self.current_node[n] = data.node_id[i]
                true = self.idle_true[data.node_id[i]]
                t = self.stamp

                with open('{}.csv'.format(n), 'a+', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(["robot","time","edge","probability (value_coeff)","sum","len","expect","true","no.of neighbours"])

                    if self.old_node[n] != 0. :
                        expect = self.average[n][self.old_node[n]]
                        writer.writerow([n,t,self.graph[self.old_node[n]][self.current_node[n]]['name'],self.value_coeff[n][self.old_node[n]][self.current_node[n]],expect,true])

                        if expect > true:
                            self.value_coeff[n][self.old_node[n]][self.current_node[n]] -= (0.2/(math.log(t)))*((expect - true)/expect)
                            #writer.writerow([n,t,self.graph[self.old_node[n]][self.current_node[n]]['name'],self.value_coeff[n][self.old_node[n]][self.current_node[n]],expect,true])
                            for m in self.graph.successors(self.old_node[n]):
                                self.value_coeff[n][self.old_node[n]][m] += (0.2/(math.log(t)))*((expect - true)/expect)/(len(list(self.graph.successors(self.old_node[n]))))
                                writer.writerow([n,t,self.graph[self.old_node[n]][m]['name'],self.value_coeff[n][self.old_node[n]][m],sum,len,expect,true,len(list(self.graph.successors(self.old_node[n])))])

                    self.idle_expect[n][self.old_node[n]][data.node_id[i]] = 0.
                self.old_node[n] = data.node_id[i]


            #for i, n in enumerate(data.robot_id):
                #self.idle_expect[n][data.node_id[i]] = 0.

            for i in enumerate(data.node_id):
                self.idle_true[i] = 0.



    def callback_next_task(self, req):
        node = req.node_done
        t = req.stamp
        bot = req.name

        #self.idle_expect[bot][node] = 0.
        #self.idle_true[node] = 0.

        neigh = list(self.graph.successors(node))
        idles = []
        for n in neigh:
            idles.append((self.average[bot][node])*(self.value_coeff[bot][node][n]))

        max_id = 0
        if len(neigh) > 1:
            max_ids = list(np.where(idles == np.amax(idles))[0])
            max_id = rn.sample(max_ids, 1)[0]

        next_walk = [node, neigh[max_id]]
        next_departs = [t]
        return NextTaskBotResponse(next_departs, next_walk)

    def callback_ready(self, req):
        algo_name = req.algo
        if algo_name == 'aneesh_mrpp3' and self.ready:
            return AlgoReadyResponse(True)
        else:
            return AlgoReadyResponse(False)

if __name__ == '__main__':
    rospy.init_node('AN_MRPP', anonymous = True)
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    done = False
    graph_name = rospy.get_param('/graph')
    num_bots = rospy.get_param ('/init_bots')
    g = nx.read_graphml(dirname + '/graph_ml/' + graph_name + '.graphml')

    s = AN_MRPP(g, num_bots)
    rospy.Subscriber('at_node', AtNode, s.callback_idle)
    rospy.Service('bot_next_task', NextTaskBot, s.callback_next_task)
    rospy.Service('algo_ready', AlgoReady, s.callback_ready)
    while not done:
        done = rospy.get_param('/done')
