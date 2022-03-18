#!/usr/bin/env python3
'''
Implementation of
@INPROCEEDINGS{8814485,  author={Asghar, Ahmad Bilal and Smith, Stephen L. and Sundaram, Shreyas},  booktitle={2019 American Control Conference (ACC)},   title={Multi-Robot Routing for Persistent Monitoring with Latency Constraints},   year={2019},  volume={},  number={},  pages={2620-2625},  doi={10.23919/ACC.2019.8814485}}
'''

import networkx as nx
import math
import numpy as np
import persistent_monitoring_one_agent as pm1

def expand_walk(paths, walk):
    '''
    Expands a given (dis)connected walk to a connected walk
    '''
    walk_e = [walk[0]]
    for i in range(1, len(walk)):
        walk_e.extend(paths[(walk[i - 1], walk[i])][1:])
    walk_e.extend(paths[(walk[-1], walk[0])][1:])
    return walk_e[:-1]

def complete_graph(graph):
    '''
    Returns complete graph and paths between nodes for a given connected graph
    '''
    paths = {}
    graph_complete = graph.copy()
    path = dict(nx.all_pairs_dijkstra_path(graph, weight = 'length'))
    for i in graph.nodes:
        for j in graph.nodes:
            if i != j and not (i, j) in graph_complete.edges():
                graph_complete.add_edge(i, j)
                graph_complete[i][j]['name'] = '{}to{}'.format(i, j)
                graph_complete[i][j]['length'] = 0
                paths[(i, j)] = path[i][j]
                for k in range(len(paths[(i, j)]) - 1):
                    graph_complete[i][j]['length'] += graph[paths[(i, j)][k]][paths[(i, j)][k + 1]]['length']
            elif i != j:
                paths[(i, j)] = [i, j]
    return (graph_complete, paths)

def robot_walks(walk, graph, paths, num_robots):
    '''
    Returns appropriate walk for each robot for a given walk and robot set
    '''
    len_walk = 0
    for i, _ in enumerate(walk):
        len_walk += graph[walk[i]][walk[(i + 1) % len(walk)]]['length']
    walk_e = expand_walk(paths, walk)
    offset = len_walk/num_robots
    rob_walks = [walk_e]
    j = 1
    len_cur = 0
    for i, _ in enumerate(walk_e):
        len_cur += graph[walk_e[i]][walk_e[(i + 1) % len(walk_e)]]['length']
        if len_cur > j * offset:
            next_walk = walk_e[(i + 1):]
            next_walk.extend(walk_e[:(i + 1)])
            rob_walks.append(next_walk)
            j += 1
        if j == num_robots:
            break
    return rob_walks
        
def cost_walk(tsp_path, graph, node_weights, num_robots):
    '''
    Returns the cost of walk 
    '''
    print(tsp_path)
    walk_nodes = set(tsp_path)
    walk_temp  = tsp_path.copy()
    walk_temp.extend(tsp_path)
    walk_temp.append(tsp_path[0])
    len_walk = 0
    for i, _ in enumerate(tsp_path):
        len_walk += graph[tsp_path[i]][tsp_path[(i + 1) % len(tsp_path)]]['length']
    latency_nodes = {n: [] for n in walk_nodes}
    cost_nodes = {n : 0 for n in walk_nodes}
    passed_nodes = [walk_temp[0]]
    latency_nodes[walk_temp[0]].append(0)
    for i in range(1, len(walk_temp)):
        cur_len = graph[walk_temp[i - 1]][walk_temp[i]]['length']
        for w in passed_nodes:
            latency_nodes[w][-1] += cur_len
        latency_nodes[walk_temp[i]].append(0)
    for n in walk_nodes:
        val = latency_nodes[n].pop(0)
        latency_nodes[n][-1] += val
        for i in range(len(latency_nodes[n])):
            latency_nodes[n][i] = min(latency_nodes[n][i], len_walk/num_robots)
        cost_nodes[n] = node_weights[n] * max(latency_nodes[n])
    return max(cost_nodes.values())

def approximation_algorithm(graph, vertex_latency):
    '''
    Approximation Algorithm function
    '''
    r_max = max(vertex_latency.values())
    r_min = min(vertex_latency.values())
    rho = r_max/r_min
    if math.log2(rho) % 1 == 0:
        rho += 1
    walks = []
    node_groups = []
    for i in range(int(math.ceil(math.log2(rho)))):
        lw = r_min * math.pow(2, i)
        hw = r_min * math.pow(2, (i + 1))
        node_groups.append([])
        for n, w in vertex_latency.items():
            if lw <= w and w < hw:
                node_groups[-1].append(n)
    
    # for i in range(int(math.ceil(math.log2(rho)))):

    pass

def latency_walks(graph, vertex_weights, num_robots):
    '''
    Latency Walk function
    '''

    gc, paths = complete_graph(graph)
    SA_tsp = nx.algorithms.approximation.simulated_annealing_tsp
    tsp = nx.algorithms.approximation.traveling_salesman_problem
    sa_tsp = lambda graph, wt: SA_tsp(graph, "greedy", weight = wt, temp = 500)

    rho = max(vertex_weights.values())/min(vertex_weights.values())
    if math.log2(rho) % 1 == 0:
        rho += 1

    node_groups = []
    for i in range(math.ceil(math.log2(rho))):
        hw = math.pow(0.5, i)
        lw = math.pow(0.5, i + 1)
        node_groups.append([])
        for n, w in vertex_weights.items():
            if lw < w and w <= hw:
                node_groups[-1].append(n)

    assigned_robots = 0
    if num_robots < math.log2(rho):
        node_robots = []
        walk_robots = []
        for j in range(num_robots):
            lw = math.ceil(j * math.log2(rho)/num_robots)
            hw = math.ceil((j + 1) * math.log2(rho)/num_robots)
            node_robots.append([])
            for i in range(lw, hw):
                node_robots[-1].extend(node_groups[i])
            if len(node_robots[-1]) > 0:
                gs = gc.subgraph(node_robots[-1])
                print (nx.is_strongly_connected(gs))
                node_weights = {n: vertex_weights[n] for n in node_robots[-1]}
                weight_max = max(node_weights.values())
                for n in node_weights.keys():
                    node_weights[n] /= weight_max
                walk_robots.append([pm1.minmax_latency_one_robot(gs, node_weights), 1])
                assigned_robots += 1

    else:
        eq_rob = math.floor(num_robots/math.log2(rho))
        walk_robots = []
        for i in range(math.ceil(math.log2(rho))):
            if len(node_groups[i]) > 0:
                walk_robots.append([tsp(gc, weight = 'length', nodes = node_groups[i], cycle = False, method = sa_tsp), eq_rob])
                assigned_robots += eq_rob
    walk_costs = []
    for rob_walk in walk_robots:
        walk_costs.append(cost_walk(rob_walk[0], gc, vertex_weights, rob_walk[1]))

    while assigned_robots < num_robots:
        max_ids = list(np.where(walk_costs == np.amax(walk_costs))[0])
        max_id = max_ids[0]
        walk_robots[max_id][1] += 1
        assigned_robots += 1
        walk_costs[max_id] = cost_walk(walk_robots[max_id][0], gc, vertex_weights, walk_robots[max_id][1])

    walks_div = []
    for rob_walk in walk_robots:
        w = robot_walks(rob_walk[0], gc, paths, rob_walk[1])
        walks_div.extend(w)

    return walks_div




