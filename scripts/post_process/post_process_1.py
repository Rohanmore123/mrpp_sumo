#!/usr/bin/env python3

'''
Input - config file path for a particular simulation
Ouput - different csv files and also append the summary to the appropriate csv file
'''

import os
import sys
import shutil
import rospkg
import yaml
import networkx as nx
import pandas as pd
import csv

def main(config_path):
    '''
    Create csv files 
    '''
    dir_name = rospkg.RosPack().get_path('mrpp_sumo')
    config_name = config_path.split('/')[-1].split('.')[0]
    with open(config_path, 'r') as f:
        config = yaml.load(f, yaml.FullLoader)

    out_str = dir_name + '/outputs/{}/'.format('ane') + config['random_string']
    sim_dir = dir_name + '/post_process/{}'.format(config['random_string'])  
    os.mkdir(sim_dir)
    shutil.copy(config_path, sim_dir)
    shutil.copy(out_str + '_visits.in', sim_dir)
    shutil.copy(out_str + '_command.in', sim_dir)

    graph = nx.read_graphml(dir_name + '/graph_ml/{}.graphml'.format(config['graph']))
    nodes = list(graph.nodes())
    edges = [graph[e[0]][e[1]]['name'] for e in list(graph.edges())]
    n = len(nodes)
    num_bots = int(config['init_bots'])
    sim_length = int(config['sim_length'])
    algo_name = config['algo_name']
    # priority_nodes = list(config['priority_nodes'].split(' '))
    # non_priority_nodes = [n for n in nodes if n not in priority_nodes]

    cols_n = ['time']
    cols_n.extend(nodes)
    # df1 = pd.DataFrame(columns = cols_n)
    cols_e = ['time']
    cols_e.extend(edges)
    # df2 = pd.DataFrame(columns = cols_e)

    all_lines = []    
    with open('{}_visits.in'.format(out_str), 'r') as f:
        all_lines = f.readlines()

    i = 0
    cur_time = 0
    cur_data_n = {}
    cur_data_e = {}
    for n in cols_n:
        cur_data_n[n] = cur_time
    for e in cols_e:
        cur_data_e[e] = cur_time
    
    with open(sim_dir + '/{}_node.csv'.format(config_name), 'w', newline = '') as f_nodes:
        with open(sim_dir + '/{}_edge.csv'.format(config_name), 'w', newline = '') as f_edges:
            node_writer = csv.DictWriter(f_nodes, fieldnames = cols_n)
            node_writer.writeheader()
            edge_writer = csv.DictWriter(f_edges, fieldnames = cols_e)
            edge_writer.writeheader()
            for l in all_lines:
                i += 1
                if i % 1000 == 0:
                    print(i, cur_time)
                if i % 3 == 1:
                    next_time = float(l.strip('\n'))
                    while cur_time < next_time:
                        # df1 = df1.append(cur_data_n, ignore_index = True)
                        # cdn = pd.DataFrame(cur_data_n, index = ['time'])
                        # df1 = pd.concat([df1, cdn], ignore_index = True)
                        # df1.reset_index()
                        node_writer.writerow(cur_data_n)

                        # df2 = df2.append(cur_data_e, ignore_index = True)
                        # cde = pd.DataFrame(cur_data_e, index = ['time'])
                        # df2 = pd.concat([df2, cde], ignore_index = True)
                        # df2.reset_index()
                        edge_writer.writerow(cur_data_e)

                        cur_time += 1
                        cur_data_n['time'] = cur_time
                        cur_data_e['time'] = cur_time
                        for n in nodes:
                            cur_data_n[n] += 1
                        
                elif i % 3 == 2:
                    cur_nodes = l.strip('\n').split(' ')
                    for n in cur_nodes:
                        cur_data_n[n] = 0
                else:
                    pass
    df1 = pd.read_csv(sim_dir + '/{}_node.csv'.format(config_name))
    df2 = pd.read_csv(sim_dir + '/{}_edge.csv'.format(config_name))
    print(df1.columns)
    df1 = df1.set_index('time')
    df2 = df2.set_index('time')

    df1['avg_idle'] = df1[nodes].mean(axis = 1)
    # df1['priority_avg_idle'] = df1[priority_nodes].mean(axis = 1)
    # df1['non_priority_avg_idle'] = df1[non_priority_nodes].mean(axis = 1)
    df1['max_idle'] = df1[nodes].max(axis = 1)
    # df1['priority_max_idle'] = df1[priority_nodes].max(axis = 1)
    # df1['non_priority_max_idle'] = df1[non_priority_nodes].max(axis = 1)
    # df1['avg_ratio'] = df1['non_priority_avg_idle']/df1['priority_avg_idle']
    df1.to_csv(sim_dir + '/{}_node.csv'.format(config_name))
    df2.to_csv(sim_dir + '/{}_edge.csv'.format(config_name))

    #Adding to master data set

    to_add = {}
    to_add = config.copy()
    to_add['avg_idle'] = df1['avg_idle'].mean()
    # to_add['priority_avg_idle'] = df1['priority_avg_idle'].mean()
    # to_add['non_priority_avg_idle'] = df1['non_priority_avg_idle'].mean()
    to_add['max_idle'] = df1['max_idle'].max()
    # to_add['priority_max_idle'] = df1['priority_max_idle'].max()
    # to_add['non_priority_max_idle'] = df1['non_priority_max_idle'].max()
    # to_add['avg_ratio'] = df1['avg_ratio'].mean()
    if not os.path.isfile(dir_name + '/{}.csv'.format(algo_name)):
        df = pd.DataFrame(columns = to_add.keys())
    else:
        df = pd.read_csv(dir_name + '/{}.csv'.format(algo_name))
    for col in to_add.keys():
        if not col in df.columns:
            df.reindex(columns = df.columns.tolist() + [col])
    if not to_add['random_string'] in map(str, df['random_string']):
        ta = pd.DataFrame(to_add, index = ['random_string'])
        df = pd.concat([df, ta], ignore_index = True)
        # df = df.append(to_add, ignore_index = True)
    df.to_csv(dir_name + '/{}.csv'.format(algo_name), index = False)
    del df1, df2, df 

if __name__ == '__main__':
    main(sys.argv[1])