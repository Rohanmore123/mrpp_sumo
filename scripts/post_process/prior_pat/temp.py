#!/usr/bin/env python3

import os
import rospkg
import glob
import sys
import yaml

def main(string):
    dir_name = rospkg.RosPack().get_path('mrpp_sumo')
    config_files = glob.glob(dir_name + '/config/{}/{}*.yaml'.format(string, string))
    output_files = glob.glob(dir_name + '/outputs/{}/{}*.in'.format(string, string))
    count = 0
    # for conf in config_files:
    
    #     with open(conf, 'r') as f:
    #         config = yaml.load(f, yaml.FullLoader)
    #     config['random_string'] += '_10'
    #     path = conf.split('/')
    #     name = path[-1].split('.')[0]
        
    #     with open(conf, 'w') as f:
    #         yaml.dump(config, f)
    #     count += 1
    #     print ('{} Done'.format(count))
    for out in output_files:    
        name = out.split('/')
        con = name[-1].split('.')[0]
        con1 = con.split('_')
        con2 = '_'.join(con1[:-1]) + '_10_' + con1[-1] + '.in'
        out1 = '/'.join(name[:-1]) + '/' + con2
        os.rename(out, out1)    
        count += 1
        print ('{} Done'.format(count))

if __name__ == '__main__':
    main(sys.argv[1])