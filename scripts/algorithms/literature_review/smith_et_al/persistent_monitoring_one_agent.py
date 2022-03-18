#!/usr/bin/env python3

'''
Implementation of -

@misc{alamdari2012persistent,
      title={Persistent Monitoring in Discrete Environments: Minimizing the Maximum Weighted Latency Between Observations}, 
      author={Soroush Alamdari and Elaheh Fata and Stephen L. Smith},
      year={2012},
      eprint={1202.5619},
      archivePrefix={arXiv},
      primaryClass={cs.DS}
}
'''

import networkx as nx
import math

def expand_walk(paths, walk):
    walk_e = [walk[0]]
    i = 0
    while i < len(walk):
        if walk[i] == walk[(i + 1) % len(walk)]:
            walk.pop(i)
        else:
            i += 1
            
    for i in range(1, len(walk)):
        walk_e.extend(paths[(walk[i - 1], walk[i])][1:])
        
    walk_e.extend(paths[(walk[len(walk) - 1], walk[0])][1:])
    return walk_e[:-1]

def complete_graph(graph):
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

def partition_path(graph, path, partitions):
    if partitions == 1:
        return [path]
    else:
        len_path = 0
        for i in range(len(path) - 1):
            len_path += graph[path[i]][path[i + 1]]['length']
        len_path /= 2
        part_len = 0
        i = 0
        while part_len < len_path:
            part_len += graph[path[i]][path[i + 1]]['length']
            i += 1
        part_a = partition_path(graph, path[:i], partitions/2)
        part_b = partition_path(graph, path[i:], partitions/2)
        part_a.extend(part_b)
        return part_a

def minmax_latency_one_robot(graph, node_weights):
    '''
    Beh jao
    '''
    SA_tsp = nx.algorithms.approximation.simulated_annealing_tsp
    tsp = nx.algorithms.approximation.traveling_salesman_problem
    sa_tsp = lambda graph, wt: SA_tsp(graph, "greedy", weight = wt, temp = 500)

    rho = max(node_weights.values())/min(node_weights.values())
    log_rho = int(math.ceil(math.log2(rho)))

    nodes = list(graph.nodes())
    if log_rho > (math.floor(math.log2(len(nodes))) + 1):
        log_rho = int(math.floor(math.log2(len(nodes))) + 1)
    
    node_groups = []
    for i in range(math.ceil(math.log2(rho)) + 1):
        hw = math.pow(0.5, i - 1)
        lw = math.pow(0.5, i)
        node_groups.append([])
        for n, w in node_weights.items():
            if lw <= w and w < hw:
                node_groups[-1].append(n)
                nodes.remove(n)
    t = int(math.pow(2, math.ceil(math.log2(rho)) + 1))
    s_walks = [[] for _ in range(t)]
    for i in range(log_rho + 1):
        if len(node_groups[i]) > 0:
            tsp_path = tsp(graph, weight = 'length', nodes = node_groups[i], cycle = False, method = sa_tsp)
            path_parts = partition_path(graph, tsp_path, math.pow(2, i))
            for j in range(t):
                s_walks[j].extend(path_parts[int(j % math.pow(2, i))])
    for i in range(t):
        s_walks[i] = tsp(graph, weight = 'length', nodes = s_walks[i], cycle = False, method = sa_tsp)
    i = 1
    for n in nodes:
        s_walks[2*i - 1].append(n)
        i += 1
    walk = s_walks[0].copy()
    for i in range(1, t):
        if walk[-1] == s_walks[i][0]:
            walk.extend(s_walks[i][1:])
        else:
            walk.extend(s_walks[i])
    return walk

def minmax_latency_wrapper(graph, node_weights):
    '''
    Returns the walk for a given graph and node_weights
    '''
    gc, paths = complete_graph(graph)
    walk_gc = minmax_latency_one_robot(gc, node_weights)
    print(len(walk_gc))
    return expand_walk(paths, walk_gc)