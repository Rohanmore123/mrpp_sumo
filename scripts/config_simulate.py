#!/bin/usr/env python3

import configparser as CP
import os
import rospkg
import glob
import rosparam

# missed = [97, 96, 95, 86, 83, 78, 44, 12]
dir_name = rospkg.RosPack().get_path('mrpp_sumo')
config_files = glob.glob(dir_name + '/config/ppa_basic_10_3/*.yaml')
count = 0
for conf in config_files:
# for i in missed:
    # conf = dir_name + '/config/latency_walks_5/latency_walks_5_{}.yaml'.format(i)
    params = rosparam.load_file(conf)[0][0]
    os.system('xterm -e "{}/tpbp.sh" {} {}'.format(dir_name, conf, params['algo_name']))
    name = conf.split('/')[-1].split('.')[0]
    # os.system('xterm -e "{}/tpbp.sh" {}/config/ran{}.yaml'.format(dir_name, dir_name, i))
    # os.system('python3 {}/scripts/algorithms/offline_algos/multi_q_tsp.py {}'.format(dir_name, conf))
    # os.system('ffmpeg -framerate 20 -i {a}/outputs/{b}/img_%d.png -c:v libx264 -pix_fmt yuv420p {a}/outputs/{b}.mp4'.format_map({'a': dir_name, 'b': name}))
    count += 1
    print ('{} Done {}'.format(count, conf))
