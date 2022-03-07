#!/usr/bin/env python3

import os
import rospkg
import glob
import sys


def main(string):
    dir_name = rospkg.RosPack().get_path('mrpp_sumo')
    config_files = glob.glob(dir_name + '/config/{}/{}*.yaml'.format(string, string))
    count = 0
    for conf in config_files:
        # path = conf.split('/')
        # name = path[-1].split('.')[0]
        print ('Processing {}'.format(conf))
        os.system('python3 {}/scripts/post_process/prior_pat/post_process_1.py {}'.format(dir_name, conf))
        count += 1
        print ('{} Done'.format(count))

if __name__ == '__main__':
    main(sys.argv[1])