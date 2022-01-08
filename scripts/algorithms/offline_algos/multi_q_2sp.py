#!/usr/bin/env python3


'''
Parameters:
1. Graph
2. Number of episodes
3. Number of threads
4. Max number of bots
5. algo_name == 'ant_q_tsp'
6. random_string

Outputs:
1. 'random_string'_vis.html - Visualization of optimal cycle across episodes
2. 'random_string'_seq.in - sequence of node visits
'''

import networkx as nx
import rospkg
import sys
import random as rn
import numpy as np
import plotly.graph_objects as go
import plotly.io as pio
pio.kaleido.scope.mathjax = None
import rosparam
import matplotlib.animation as anplot
import matplotlib.pyplot as plt
import os

class Multi_Q:

    def __init__(self, graph, time_period, num_threads, sim_dir, q_0 = 0.8, alpha = 0.1, gamma = 0.3, delta = 3, beta = 1, W = 10000):
        self.graph = graph.copy()
        self.edges_og = list(graph.edges())
        self.nodes = list(graph.nodes())
        self.sim_dir = sim_dir
        self.time_period = time_period
        self.paths = {}
        self.num_threads = min(num_threads, len(self.nodes))
        for i in self.nodes:
            for j in self.nodes:
                if i == j and not (i, j) in self.graph.edges():
                    self.graph.add_edge(i, j)
                    self.graph[i][j]['name'] = '{}to{}'.format(i, j)
                    self.graph[i][j]['length'] = self.time_period
                    self.paths[(i, j)] = [i, j]
                elif i != j and not (i, j) in self.edges_og:
                    self.graph.add_edge(i, j)
                    self.graph[i][j]['name'] = '{}to{}'.format(i, j)
                    self.graph[i][j]['length'] = nx.dijkstra_path_length(graph, i, j, 'length')
                    self.paths[(i, j)] = nx.dijkstra_path(graph, i, j, 'length')
                elif i != j:
                    self.paths[(i, j)] = [i, j]

        self.edges = list(self.graph.edges())
        self.lengths = [float(self.graph[e[0]][e[1]]['length']) for e in self.edges]
        avg_len = sum(self.lengths)/len(self.edges)
        max_len = max(self.lengths)
        for e in self.edges:
            self.graph[e[0]][e[1]]['h_value'] = max_len/float(self.graph[e[0]][e[1]]['length'])
            self.graph[e[0]][e[1]]['q_value'] = max_len/(avg_len)
        self.q = q_0
        self.q_rem = 1 - q_0
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta = delta
        self.W = sum(self.lengths)
        self.best_walk = []
        self.best_len = np.inf
        self.best_walk_directions = []
        self.best_walk_edges = []
        self.robs = np.inf
        self.init_plot()    

    def init_plot(self):

        d = 20 #radius of nodes

        self.edge_traces = [] 
        self.edges_plot = []
        for edge in self.edges_og:
            if not (edge[1], edge[0]) in self.edges_plot:
                self.edges_plot.append(edge)
                x0, y0 = self.graph.nodes[edge[0]]['x'], self.graph.nodes[edge[0]]['y']
                x1, y1 = self.graph.nodes[edge[1]]['x'], self.graph.nodes[edge[1]]['y']
                self.edge_traces.append(go.Scatter(x=[x0, x1, None], y=[y0, y1, None], line=dict(width=0.5, color = 'blue'), opacity = 1, hoverinfo='none', mode='lines'))
        
        node_x = []
        node_y = []

        for node in list(self.nodes):
            x, y = self.graph.nodes[node]['x'], self.graph.nodes[node]['y']
            node_x.append(x)
            node_y.append(y)

        self.node_trace = go.Scatter(
            x=node_x, y=node_y,
            mode='markers',
            # hoverinfo='text',
            marker=dict(
                showscale=False,
                reversescale=False,
                color='white',
                size=d,
                line_width=0))

        fig = go.Figure(data=[self.node_trace] + self.edge_traces,
            layout=go.Layout(
            title='Graph \'{}\''.format(graph_name),
            title_x = 0.4,
            # titlefont_size=16,
            showlegend=False,
            hovermode='closest',
            margin=dict(b=20,l=5,r=5,t=40),
            xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            yaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            plot_bgcolor='black'
            # updatemenus=[dict(type="buttons", buttons=[dict(label="Play", method="animate", args=[None, {"frame": {"duration": 5, "redraw": True}}])])]),
            # frames = test.frames
            ))
        fig.update_yaxes(scaleanchor = 'x', scaleratio = 1)
        fig.to_image(format="png", engine="kaleido")
        fig.write_image('{}/img_0.png'.format(self.sim_dir))

    
    def run(self):
        actions = self.edges.copy()
        in_degree = [False for n in self.nodes]
        out_degree = [False for n in self.nodes]
        edge_sets = []
        edge_lens = []
        cur_edges = []
        cur_len = 0
        robots = []
        nodes_rem = self.nodes.copy()

        while not (all(in_degree) and all(out_degree)):
            # print(self.nodes, nodes_rem)
            # print(in_degree)
            # print(out_degree)
            # print(actions) 
            # print(edge_sets)

            if len(cur_edges) == 0:
                # print('here')
                start_node = rn.sample(nodes_rem, 1)[0]
                cur_node = start_node
                cur_actions = []
                for a in actions:
                    if a[0] == cur_node:
                        cur_actions.append(a) 
            #     print(cur_node, cur_actions)
            # print(cur_edges)
            # print(start_node, cur_node, cur_actions)  
            # print('--------')


 
            values = [(self.graph[act[0]][act[1]]['q_value'] ** self.delta) * (self.graph[act[0]][act[1]]['h_value'] ** self.beta) for act in cur_actions]
            tot_val = sum(values)
            values = [v/tot_val for v in values]

            if rn.random() <= self.q:
                act = cur_actions[np.argmax(values)]

            else:
                #random-proportional
                r_val = rn.random()
                cum_sum = 0.
                i = -1
                while r_val < cum_sum:
                    i += 1
                    cum_sum += values[i]
                act = cur_actions[i]

            actions.remove(act)

            out_degree[self.nodes.index(act[0])] = True
            in_degree[self.nodes.index(act[1])] = True

            for n in self.graph.successors(act[0]):
                if (act[0], n) in actions:
                    actions.remove((act[0], n))

            for n in self.graph.predecessors(act[1]):
                if (n, act[1]) in actions:
                    actions.remove((n, act[1]))


            path = self.paths[act]
            for i in range(1, len(path)):
                if path[i] in nodes_rem:
                    nodes_rem.remove(path[i])
                cur_edges.append((path[i - 1], path[i]))
            cur_len += self.graph[act[0]][act[1]]['length']
            
            if len(path) > 2:
                for i in range(1, len(path) - 1):
                    out_degree[self.nodes.index(path[i])] = True
                    for n in self.graph.successors(path[i]):
                        if ((path[i], n) in actions) and (n != start_node):
                            actions.remove((path[i], n))
                    if path[i] != start_node:
                        in_degree[self.nodes.index(path[i])] = True

                        for n in self.graph.predecessors(path[i]):
                            if (n, path[i]) in actions:
                                actions.remove((n, path[i]))


            # for i in range(len(path) - 1):
            #     edges_in_run[path[i]] = (path[i], path[i + 1])
            cur_node = act[-1]
            if cur_node == start_node:
                edge_sets.append(cur_edges)
                edge_lens.append(cur_len)
                cur_edges = []
                robots.append(np.ceil(cur_len/self.time_period))
                cur_len = 0

            if actions:
                cur_actions = []
                for a in actions:
                    if a[0] == cur_node:
                        cur_actions.append(a)
                if len(cur_actions) > 0:   
                    next_val = max([self.graph[nex[0]][nex[1]]['q_value'] for nex in cur_actions])
                    self.graph[act[0]][act[1]]['q_value'] *= (1 - self.alpha)
                    self.graph[act[0]][act[1]]['q_value'] += self.alpha * self.gamma * next_val 


        return edge_sets, edge_lens, robots
    

    # def cost_of_walks(self, edg):
    #     # print(edg)
    #     remain_edges = edg.copy()
    #     edge_sets = [] 
    #     edge_lens = []
    #     cur_edges = []
    #     cur_len = 0
    #     robots = []
    #     nodes_rem = self.nodes.copy()
    #     while nodes_rem:
    #         if len(cur_edges) == 0:
    #             start_node = nodes_rem[0]
    #             cur_edges.append(remain_edges[start_node])
    #             cur_node = cur_edges[-1][1]
    #             cur_len += self.graph[cur_edges[-1][0]][cur_edges[-1][1]]['length'] 
    #         else:
    #             if cur_node == start_node:
    #                 nodes_rem.remove(cur_node)
    #                 edge_sets.append(cur_edges)
    #                 edge_lens.append(cur_len)
    #                 cur_edges = []
    #                 robots.append(np.ceil(cur_len/self.time_period))
    #                 cur_len = 0
    #             else:
    #                 nodes_rem.remove(cur_node)
    #                 cur_edges.append(remain_edges[cur_node])
    #                 cur_node = cur_edges[-1][-1]
    #                 cur_len += self.graph[cur_edges[-1][0]][cur_edges[-1][1]]['length']

    #     # print(edge_sets, edge_lens, robots)
    #     return edge_sets, edge_lens, robots

    def episode(self, ep_count):
        walks = []
        lens = []
        costs = []
        robs = []
        if ep_count > 100 and self.q < 0.9:
            self.q += (self.q_rem/2) ** 2
            self.q_rem = 1 - self.q 

        for i in range(self.num_threads):
            #dictionary of node to outgoing edge
            # print(ep_count, i, edges_run)
            es, ls, rob = self.run()

            cost = 0
            for i, c in enumerate(ls):
                cost += (rob[i] * self.time_period - c) ** 2
            cost *= sum(rob) ** 2
            
            walks.append(es)
            lens.append(ls)
            costs.append(cost)
            robs.append(sum(rob))

        iter_best_walk = walks[np.argmin(robs)]
        iter_best_count = min(robs)
        iter_best = costs[np.argmin(robs)]
        iter_best_edges = []
        for w in iter_best_walk:
            # if len(w) != 1:
            for a in w:
                path = self.paths[(a[0], a[1])]
                for j in range(len(path) - 1):
                    iter_best_edges.append((path[j], path[j + 1]))


        if iter_best_count < self.robs or (iter_best_count == self.robs and iter_best < self.best_len):
            self.best_len = iter_best
            self.best_walk = iter_best_walk
            self.best_walk_directions = iter_best_edges
            self.best_walk_edges = []
            self.robs = iter_best_count
            for i in self.best_walk_directions:
                # path = self.paths[(self.best_walk[i], self.best_walk[i + 1])]
                # for j in range(len(path) - 1):
                    if i[0] == i[1]:
                        pass
                    elif i in self.edges_plot:
                        self.best_walk_edges.append(i)
                    else:
                        self.best_walk_edges.append((i[1], i[0]))

        del_aq = len(self.nodes)/(iter_best_count)
        edge_counts = {tuple(e): 0 for e in self.edges_plot}
        for walk in walks:
            for circuit in walk:
                for e in circuit:
                    self.graph[e[0]][e[1]]['q_value'] =  self.graph[e[0]][e[1]]['q_value'] + self.alpha * del_aq
                    path = self.paths[e]
                    for j in range(len(path) - 1):
                        if path[j] == path[j + 1]:
                            pass
                        elif (path[j], path[j + 1]) in self.edges_plot:
                            edge_counts[(path[j], path[j + 1])] += 1
                        else:
                            edge_counts[(path[j + 1], path[j])] += 1

 
        if ep_count % 10 == 0:
            max_count = max(list(edge_counts.values()))
            edge_traces = self.edge_traces.copy()

            for i in range(len(self.edges_plot)):
                edge_traces[i].line.width = 1
                edge_traces[i].line.color = 'white'
                if max_count > 0:
                    edge_traces[i].opacity = min(edge_counts[tuple(self.edges_plot[i])]/max_count, 1)
                    if tuple(self.edges_plot[i]) in self.best_walk_edges:
                        edge_traces[i].line.color = 'orange'
                        edge_traces[i].opacity = 1
                    edge_traces[i].line.width = 4
                else:
                    edge_traces[i].opacity = 0.

            fig = go.Figure(data=[self.node_trace] + edge_traces,
                layout=go.Layout(
                title='Graph \'{}\', Episode - {}, Robots - {}'.format(graph_name, ep_count, self.robs),
                title_x = 0.4,
                # titlefont_size=16,
                showlegend=False,
                hovermode='closest',
                margin=dict(b=20,l=5,r=5,t=40),
                xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
                yaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
                plot_bgcolor='black'
                # updatemenus=[dict(type="buttons", buttons=[dict(label="Play", method="animate", args=[None, {"frame": {"duration": 5, "redraw": True}}])])]),
                # frames = test.frames
                ))
            fig.update_yaxes(scaleanchor = 'x', scaleratio = 1)
            fig.to_image(format="png", engine="kaleido")
            fig.write_image('{}/img_{}.png'.format(self.sim_dir, int(ep_count//10)))

        return (iter_best_edges, iter_best_count)



if __name__ == '__main__':
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    sim_file = sys.argv[1]

    params = rosparam.load_file(sim_file)[0][0]
    graph_name = params['graph']
    num_episodes = int(params['num_episodes'])
    num_threads = int(params['num_threads'])
    algo = params['algo_name']
    sim_name = params['random_string']
    time_period = float(params['time_period']) * 10 # velocity assumed to be 10 m/s
    sim_dir = dirname + '/outputs/' + sim_name
    os.mkdir(sim_dir)

    if algo != 'multi_q_2sp':
        print('Not this algorithm')
        exit

    g = nx.read_graphml(dirname + '/graph_ml/' + graph_name + '.graphml')
    
    with open(dirname + '/outputs/{}_visit_seq.in'.format(sim_name), 'w') as f:
        test = Multi_Q(g, time_period, num_threads, sim_dir)
        for i in range(num_episodes):

            w, l = test.episode(i + 1)
            print(i, l)
            
            f.write('Epsiode' + str(i + 1) + ': ' + str(l) + '\n')
            f.write('\n')
            for n in w:
                f.write(str(n)+ '\n')
            f.write('\n')

        f.write('Minimum Cost: ' + str(test.best_len) + '\n')
        f.write('\n')
        for n in test.best_walk:
            f.write(str(n)+ '\n')