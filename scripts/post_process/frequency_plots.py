# from turtle import color
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
    name_list =name.split('_')
    sim_dir = dirname + '/post_process/' + name
    # with open('{}/config/{}_{}/{}.yaml'.format(dirname,"_".join(name_list[:-2]),name_list[-1],name), 'r') as f:
    with open('{}/config/{}/{}.yaml'.format(dirname,"_".join(name_list[:-1]),name), 'r') as f:
        config = yaml.load(f, yaml.FullLoader)
    graph = nx.read_graphml(dirname + '/graph_ml/{}.graphml'.format(config['graph']))
    nodes = list(graph.nodes())
    # edges = [graph[e[0]][e[1]]['name'] for e in list(graph.edges())]
    priority_nodes = config['priority_nodes'].split(' ')
    time_period = config['time_periods'].split(' ')
    n = len(nodes)
    sim_length = int(config['sim_length'])
    # num_bots = int(config['init_bots'])
    # sim_length = int(config['sim_length'])
    # algo_name = config['algo_name']
    df1 = pd.read_csv(sim_dir + '/{}_node.csv'.format(name))
    non_priority_nodes = [u for u in graph.nodes if u not in priority_nodes]
    node_x = []
    node_y = []
    node_x_priority= []
    node_y_priority= []
    nodes = list(graph.nodes)
    max_idle=[]
    avg_idle=[]
    avg_idle_pn=[]
    max_idle_pn=[]
    # print(non_priority_nodes,priority_nodes)
    for node in graph.nodes():
        x, y = graph.nodes[node]['x'], graph.nodes[node]['y']
        if node in priority_nodes:
            node_x_priority.append(x)
            node_y_priority.append(y)
            avg_idle.append(df1[node].mean())
            max_idle.append(df1[node].max())
            avg_idle_pn.append(df1[node].mean())
            max_idle_pn.append(df1[node].max())

        else:
            node_x.append(x)
            node_y.append(y)
            avg_idle.append(df1[node].mean())
            max_idle.append(df1[node].max())

    d = 20 #radius of nodes
    edge_vis = {'cair': d/2, 'circle': d/2, 'grid_5_5': d/2, 'iitb': d, 'ladder': d/2, 'st_line': d, 'st_line_assym': d}
    s = int(edge_vis[config['graph']])
    # print(s)
    edge_x = []
    edge_y = []
    for edge in graph.edges():
        x0, y0 = graph.nodes[edge[0]]['x'], graph.nodes[edge[0]]['y']
        x1, y1 = graph.nodes[edge[1]]['x'], graph.nodes[edge[1]]['y']
        edge_x.append(x0)
        edge_x.append(x1)
        edge_x.append(None)
        edge_y.append(y0)
        edge_y.append(y1)
        edge_y.append(None)
    
    idleness_over=[]
    df4={}
    # df4=df1
    for n in priority_nodes:
        print(n)
        # idleness_over.append(df1[n].value_counts()[1])
        # for i in df1[n]:
        df4[n] = df1[n]-float(time_period[0])
    # print(i)
    df4=pd.DataFrame(df4)
    df4[df4<0]=0
    df4.to_csv(sim_dir + '/{}_overshoot.csv'.format(name),index=False)

    for n in df4.columns:
        idleness_over.append(df4[n].value_counts()[1]/df1[n].value_counts()[0])
    # print(idleness_over)

    #plotting priority node locations
    def plot_priority_nodes(node_x,node_y,node_x_priority,node_y_priority):
        non_priority = go.Scatter(
            x=node_x, y=node_y,
            mode='markers',
            hoverinfo='text',
            marker=dict(
                showscale=False,
                # colorscale options
                #'Greys' | 'YlGnBu' | 'Greens' | 'YlOrRd' | 'Bluered' | 'RdBu' |
                #'Reds' | 'Blues' | 'Picnic' | 'Rainbow' | 'Portland' | 'Jet' |
                #'Hot' | 'Blackbody' | 'Earth' | 'Electric' | 'Viridis' |
                colorscale='Blues',
                reversescale=False,
                color=1,
                size=2 * d,
                # opacity = 0.5,
                # showscale = False,
                # colorbar=dict(
                #     thickness=15,
                #     title='Node Connections',
                #     xanchor='left',
                #     titleside='right'
                # ),
                line_width=0))
        priority = go.Scatter(
        x=node_x_priority, y=node_y_priority,
        mode='markers',
        hoverinfo='text',
        marker=dict(
            showscale=False,
            # colorscale options
            #'Greys' | 'YlGnBu' | 'Greens' | 'YlOrRd' | 'Bluered' | 'RdBu' |
            #'Reds' | 'Blues' | 'Picnic' | 'Rainbow' | 'Portland' | 'Jet' |
            #'Hot' | 'Blackbody' | 'Earth' | 'Electric' | 'Viridis' |
            colorscale='Reds',
            reversescale=False,
            color=1,
            size=2 * d,
            # opacity = 0.5,
            # showscale = False,
            # colorbar=dict(
            #     thickness=15,
            #     title='Node Connections',
            #     xanchor='left',
            #     titleside='right'
            # ),
            line_width=0))

        fig = go.Figure(data=[non_priority,priority],
                    layout=go.Layout(
                    # title='Graph \'{}\''.format(config['graph']),
                    # titlefont_size=16,
                    showlegend=True,
                    hovermode='closest',
                    margin=dict(b=20,l=5,r=0,t=40),
                    xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
                    yaxis=dict(showgrid=False, zeroline=False, showticklabels=False), 
                    width=1200,
                    height=1000)
                    )

        # fig.add_trace(go.Heatmap(x = node_x, y = node_y, z = node_z))
        for i in range(0, len(edge_x), 3):
            # print (edge_x[i])
            x1 = edge_x[i + 1]  # arrows' head
            y1 = edge_y[i + 1]  # arrows' head
            x0 = edge_x[i]  # arrows' tail
            y0 = edge_y[i]  # arrows' tail
            # print (x0, y0, x1, y1)

            vert = True
            if x0 != x1:
                m = (y1 - y0)/(x1 - x0)
                c = y0 - m * x0
                vert = False
            
            if vert:
                yt = y0 + s * np.sign(y1 - y0)
                yh = y1 - s * np.sign(y1 - y0)
                xt = x0
                xh = x1
            else:
                if y1 == y0:
                    xt = x0 + s * np.sign(x1 - x0)
                    xh = x1 - s * np.sign(x1 - x0)
                    yt = y0
                    yh = y1
                else:
                    xt = x0 + math.sqrt(s ** 2 / (m ** 2 + 1)) * np.sign(x1 - x0)
                    xh = x1 - math.sqrt(s ** 2 / (m ** 2 + 1)) * np.sign(x1 - x0)
                    yt = m * xt + c
                    yh = m * xh + c
            

            fig.add_annotation(
                x=xh,  # arrows' head
                y=yh,  # arrows' head
                ax=xt,  # arrows' tail
                ay=yt,  # arrows' tail
                xref='x',
                yref='y',
                axref='x',
                ayref='y',
                text='',  # if you want only the arrow
                showarrow=True,
                arrowhead=1,
                arrowsize=2,
                arrowwidth=1,
                arrowcolor='black'
                )
        fig.update_yaxes(
            scaleanchor = "x",
            scaleratio = 1,
        )
        fig.update_layout(title_text='Priority Node Loctions', title_x=0.5, titlefont_size = 20, plot_bgcolor = 'rgba(0, 0, 0, 0)')
        fig.to_image(format="png", engine="kaleido")
        fig.write_image('{}/{}_Priority_nodes.png'.format(sim_dir, name))
        del fig
        # fig.show()

    #ploting spatial graph of idleness overshoot
    def idleness_spatial():
            node_trace2 = go.Scatter(
            x=node_x, y=node_y,
            mode='markers',
            hoverinfo='text',
            marker=dict(
                showscale=True,
                # colorscale options
                #'Greys' | 'YlGnBu' | 'Greens' | 'YlOrRd' | 'Bluered' | 'RdBu' |
                #'Reds' | 'Blues' | 'Picnic' | 'Rainbow' | 'Portland' | 'Jet' |
                #'Hot' | 'Blackbody' | 'Earth' | 'Electric' | 'Viridis' |
                colorscale='Blackbody',
                reversescale=False,
                color=[],
                size=2 * d,
                opacity = 0.2,
                # showscale = False,
                # colorbar=dict(
                #     thickness=15,
                #     title='Node Connections',
                #     xanchor='left',
                #     titleside='right'
                # ),
                line_width=0))

            node_trace3 = go.Scatter(
            x=node_x_priority, y=node_y_priority,
            mode='markers',
            hoverinfo='text',
            marker=dict(
                showscale=True,
                # colorscale options
                #'Greys' | 'YlGnBu' | 'Greens' | 'YlOrRd' | 'Bluered' | 'RdBu' |
                #'Reds' | 'Blues' | 'Picnic' | 'Rainbow' | 'Portland' | 'Jet' |
                #'Hot' | 'Blackbody' | 'Earth' | 'Electric' | 'Viridis' |
                colorscale='Blues',
                reversescale=False,
                color=idleness_over,
                size=2 * d,
                opacity = 1,
                # showscale = False,
                # colorbar=dict(
                #     thickness=15,
                #     title='Node Connections',
                #     xanchor='left',
                #     titleside='right'
                # ),
                line_width=0))

            fig = go.Figure(data=[node_trace2,node_trace3],
                    layout=go.Layout(
                    # title='Graph \'{}\''.format(config['graph']),
                    # titlefont_size=16,
                    showlegend=False,
                    hovermode='closest',
                    margin=dict(b=20,l=5,r=0,t=40),
                    xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
                    yaxis=dict(showgrid=False, zeroline=False, showticklabels=False), 
                    width=1200,
                    height=1000)
                    )

            # fig.add_trace(go.Heatmap(x = node_x, y = node_y, z = node_z))
            for i in range(0, len(edge_x), 3):
                # print (edge_x[i])
                x1 = edge_x[i + 1]  # arrows' head
                y1 = edge_y[i + 1]  # arrows' head
                x0 = edge_x[i]  # arrows' tail
                y0 = edge_y[i]  # arrows' tail
                # print (x0, y0, x1, y1)

                vert = True
                if x0 != x1:
                    m = (y1 - y0)/(x1 - x0)
                    c = y0 - m * x0
                    vert = False
                
                if vert:
                    yt = y0 + s * np.sign(y1 - y0)
                    yh = y1 - s * np.sign(y1 - y0)
                    xt = x0
                    xh = x1
                else:
                    if y1 == y0:
                        xt = x0 + s * np.sign(x1 - x0)
                        xh = x1 - s * np.sign(x1 - x0)
                        yt = y0
                        yh = y1
                    else:
                        xt = x0 + math.sqrt(s ** 2 / (m ** 2 + 1)) * np.sign(x1 - x0)
                        xh = x1 - math.sqrt(s ** 2 / (m ** 2 + 1)) * np.sign(x1 - x0)
                        yt = m * xt + c
                        yh = m * xh + c
                

                fig.add_annotation(
                    x=xh,  # arrows' head
                    y=yh,  # arrows' head
                    ax=xt,  # arrows' tail
                    ay=yt,  # arrows' tail
                    xref='x',
                    yref='y',
                    axref='x',
                    ayref='y',
                    text='',  # if you want only the arrow
                    showarrow=True,
                    arrowhead=1,
                    arrowsize=2,
                    arrowwidth=1,
                    arrowcolor='black'
                    )
            fig.update_yaxes(
                scaleanchor = "x",
                scaleratio = 1,
            )
            fig.update_layout(title_text='Overshoot frequency', title_x=0.5, titlefont_size = 20, plot_bgcolor = 'rgba(0, 0, 0, 0)')
            fig.to_image(format="png", engine="kaleido")
            fig.write_image('{}/{}_Overshoot_frequency.png'.format(sim_dir, name))
            del fig    
        
    #Overshoot ratio,avg,max
    df4[df4>0]=1
    overshoot_ratio=[]
    for n in priority_nodes:
        overshoot_ratio.append(df4[n].sum()/len(df4.index))
    # print(df4)
    time_period_violation_ratio = df4.max(axis=1).sum()/len(df4.index)
    average_time_period_ratio = df4.sum(axis=1).sum()/len(df4.index)
    print(time_period_violation_ratio)
    
    def write_csv():
        # df = pd.read_csv(dirname + '/'+"_".join(name_list[:-2])+'_{}.csv'.format(str(config['depth'])))
        df = pd.read_csv(dirname + '/'+"_".join(name_list[:-1])+'.csv')
        to_add = {}
        to_add = config.copy()
        to_add['overshoot_avg'] = np.mean(overshoot_ratio)
        to_add['overshoot_max'] = np.max(overshoot_ratio)
        to_add['max_idle'] = np.max(max_idle)
        to_add['avg_idle'] = np.mean(avg_idle)
        to_add['max_idle_pn'] = np.max(max_idle_pn)
        to_add['avg_idle_pn'] = np.mean(avg_idle_pn)
        to_add['time_period_violation_ration'] = time_period_violation_ratio
        to_add['average_time_period_ratio'] = average_time_period_ratio
        for col in to_add.keys():
            if not col in df.columns:
                df.reindex(columns = df.columns.tolist() + [col])
        if not to_add['random_string'] in map(str, df['random_string']):
            df = df.append(to_add, ignore_index = True)
            # df.loc[df['random_string'] == to_add['random_string']]
        # print(df)
        # df.to_csv(dirname + '/'+"_".join(name_list[:-2])+'_{}.csv'.format(str(config['depth'])), index = False)
        df.to_csv(dirname + '/'+"_".join(name_list[:-1])+'.csv', index = False)
    # del df1, df4

    def temporal_idle():
        #scatter plot of idlness over
        plt.figure()
        sns.set_style('white')
        sns.set_context(font_scale= 1, rc = {"font.size" : 15, "axes.titlesize" : 20})
        # plt.subplots(figsize = (10, 20))
        # plt.subplots_adjust(top= 0.2)
        # sns.set(rc = {'figure.figsize':(20, 100)})
        for n in nodes:
            if n in priority_nodes:
                sns.scatterplot(data= df1.loc[::1000,n],color = 'red',alpha = 1)
            else:
                sns.scatterplot(data= df1.loc[::1000,n],color = 'blue',alpha = 0.2)

        plt.suptitle('Node Idleness Values vs Time', size = 18, y = 1.02, x = 0.4)
        sns.lineplot(data = df1.iloc[::1000], x = list(range(0,len(df1.iloc[::1000])*1000,1000)), y = df1.loc[::1000, nodes].mean(axis = 1), legend = False, linewidth = 3)
        sns.lineplot(data = df1.iloc[::1000], x = list(range(0,len(df1.iloc[::1000])*1000,1000)), y = 80, legend = False, linewidth = 3, alpha  = 0.6)
        plt.xticks(rotation = 30)
        plt.ylabel('Node Idleness')
        plt.xlabel("time in seconds")

        plt.savefig('{}/{}_temporal_idle.png'.format(sim_dir, name), bbox_inches='tight')

    # idleness_spatial()

    # plot_priority_nodes(node_x,node_y,node_x_priority,node_y_priority)
    # temporal_idle()
    write_csv()
    # temporal_idle()
if __name__ == '__main__':
    main(sys.argv[1:])
