#!/usr/bin/env python3

import os
import rospkg
import glob
import sys
from multiprocessing import Pool

def main(string):
    dir_name = rospkg.RosPack().get_path('mrpp_sumo')
    config_files = glob.glob(dir_name + '/config/{}/{}*.yaml'.format(string,string))
    count = 0
    for conf in config_files:
        # path = conf.split('/')
        # name = path[-1].split('.')[0]
        print ('Processing {}'.format(conf))
        os.system('python3 {}/scripts/post_process/prior_pat/post_process_1.py {}'.format(dir_name, conf))
        # os.system('python3 {}/scripts/post_process/frequency_plots.py {} grid_5_5'.format(dir_name, name))
        count += 1
        print ('{} Done'.format(count))

if __name__ == '__main__':
    post=['ppa_basic_0_10_3', 'ppa_basic_0_10_0', 'ppa_basic_0_10_5', 'ppa_basic_0_100_0', 'ppa_basic_0_100_3', 'ppa_basic_0_100_5']
    for i in post:
        main(i)
    #using 6 cores for processing 
    # with Pool(6) as p:
    #     p.map(main,post)
