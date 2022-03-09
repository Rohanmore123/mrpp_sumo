#!/usr/bin/env python3

import networkx as nx
import math
import numpy as np

def graph_l(graph, l):
    gc = graph.copy()
    for e in graph.edges():
        if graph[e[0]][e[1]]['length'] > l:
            gc.remove_edge(e[0], e[1])

    return gc

def MCCP2(graph, l):
    gc = graph_l(graph, l)