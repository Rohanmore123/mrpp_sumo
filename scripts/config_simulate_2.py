#!/bin/usr/env python3

import configparser as CP
import os
import rospkg
import glob
import rosparam

# missed = [175,169,163,157,151,115,109,103,55,49,43,37]
dir_name = rospkg.RosPack().get_path('mrpp_sumo')
config_files = glob.glob(dir_name + '/config/mqp*.yaml')
count = 0
for conf in config_files:
# for i in missed:
    params = rosparam.load_file(conf)[0][0]
    # os.system('xterm -e "{}/tpbp.sh" {} {}'.format(dir_name, conf, params['algo_name']))
    # name = conf.split('/')[-1].split('.')[0]
    # os.system('xterm -e "{}/tpbp.sh" {}/config/ran{}.yaml'.format(dir_name, dir_name, i))
    os.system('python3 {}/scripts/algorithms/offline_algos/multi_q_2sp.py {}'.format(dir_name, conf))
    # os.system('ffmpeg -framerate 20 -i {a}/outputs/{b}/img_%d.png -c:v libx264 -pix_fmt yuv420p {a}/outputs/{b}.mp4'.format_map({'a': dir_name, 'b': name}))
    count += 1
    print ('{} Done {}'.format(count, conf))
