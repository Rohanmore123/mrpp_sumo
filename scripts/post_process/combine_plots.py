from turtle import color
import seaborn as sns
import networkx as nx
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import rospkg
import pandas as pd
import numpy as np
import yaml
import math
import matplotlib.cm as cm
import matplotlib
import os, sys, shutil
import plotly
import kaleido
import plotly.io as pio



def main(param):
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    name = param[0]
    g = param[1]
    # edges = [graph[e[0]][e[1]]['name'] for e in list(graph.edges())]

    # tpbp_df=pd.read_csv(dirname+'/{}.csv'.format(name))
    plt.figure()
    sns.set_style('white')
    sns.set_context(font_scale= 1, rc = {"font.size" : 15, "axes.titlesize" : 20})
    # for i in tpbp_df['random_string']:
    # for i in range(1,12,3):
    # algos=['tpbp_final','tpbp_alt1_1','tpbp_util','tpbp_util1_1']
    # algos = ['tpbp_util1_1','tpbp_util1_3','tpbp_util1_5','tpbp_util']
    # algos = ['through_modified_basic_1','through_modified_basic_3','through_modified_basic_5']
    algos = ['through_basic_5']
    # algos = ['tpbp_final', 'tpbp_util', 'through_modified_FHUM_5', 'through_modified_basic_5']
    colors =['red','blue','green']
    fig,axes = plt.subplots(3,4,figsize=(30,15))
    i=24
    for j in range(4):
        for k in range(3):
            i += 1
            for z,a in enumerate(algos):
                if len(a.split("_"))==3:
                    with open('{}/config/old_config/{}/{}_{}_{}.yaml'.format(dirname, a, "_".join(a.split('_')[:-1]), str(i),a.split('_')[-1]), 'r') as f:
                        config = yaml.load(f, yaml.FullLoader)
                    sim_dir = dirname + '/post_process/' + "_".join(a.split('_')[:-1]) + "_" + str(i) + '_' + str(config['depth'])
                    df1 = pd.read_csv(sim_dir + '/{}_{}_{}_node.csv'.format("_".join(a.split('_')[:-1]), str(i), str(config['depth'])))
                else:
                    with open('{}/config/{}/{}.yaml'.format(dirname,a,a + "_" + str(i)), 'r') as f:
                        config = yaml.load(f, yaml.FullLoader)
                    sim_dir = dirname + '/post_process/' + a + "_" + str(i)
                    df1 = pd.read_csv(sim_dir + '/{}_{}_node.csv'.format(a,str(i)))
                graph = nx.read_graphml(dirname + '/graph_ml/{}.graphml'.format(config['graph']))
                nodes = list(graph.nodes())
                priority_nodes = config['priority_nodes'].split(' ')
                non_priority_nodes = [u for u in graph.nodes if u not in priority_nodes]
                time_period = config['time_periods'].split(' ')
                # for n in priority_nodes:
                    # sns.scatterplot(ax=axes[k,j],data= df1.loc[::1000,n], color = colors[z], alpha = 1)
                sns.lineplot(ax=axes[k,j],data = df1.iloc[::1000], x = list(range(0,len(df1.loc[::1000])*1000,1000)), y = df1.loc[::1000, priority_nodes].mean(axis = 1), legend = True, linewidth = 3,alpha=1)#,color = colors[z])
                sns.lineplot(ax=axes[k,j],data = df1.iloc[::1000], x = list(range(0,len(df1.loc[::1000])*1000,1000)), y = df1.loc[::1000, nodes].mean(axis = 1), legend = True, linewidth = 3,alpha=1)#,color = colors[z])
                sns.lineplot(ax=axes[k,j],data = df1.iloc[::1000], x = list(range(0,len(df1.loc[::1000])*1000,1000)), y = df1.loc[::1000, non_priority_nodes].mean(axis = 1), legend = True, linewidth = 3,alpha=1)#,color = colors[z])
                # sns.lineplot(ax=axes[k,j],data = df1.iloc[::1000], x = list(range(0,len(df1.loc[::1000])*1000,1000)), y = df1.loc[::1000, priority_nodes].mean(axis = 1), legend = True, linewidth = 3)
            axes[k,j].set_title("{} robots and {} priority nodes".format(config['init_bots'],k+4))
            axes[k,j].set_ylim(0,500)
            # axes[k,j].set_xlim(0)
            sns.lineplot(ax=axes[k,j],data = df1.iloc[::1000], x = list(range(0,len(df1.loc[::1000])*1000,1000)), y = float(time_period[0]), legend = True, linewidth = 3,alpha=0.4)
            # algo_name=['tpbp_modified_basic','tpbp_modified_FHUM','depth_modified_basic','depth_modified_FHUM','through_modified_basic','through_modified_FHUM']
            axes[k,j].legend(['Pn_avg','AN_avg','NPN_avg'],prop={'size': 9})
            # axes[k,j].set_xlabel('time in seconds')
            plt.xticks(rotation = 30)
            # plt.ylabel('Node Idleness')
            # plt.xlabel("time in seconds")
    plt.suptitle('all PNode avg Idlesness vs Time,through plots with c2=10', size = 40, y = 1.02, x = 0.4)
    plt.savefig('{}/test_{}.png'.format(dirname, str(i/12)), bbox_inches='tight')



if __name__ == '__main__':
    main(['tpbp_final','grid_5_5'])
