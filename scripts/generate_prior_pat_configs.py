#!/bin/usr/env python3

'''
A script to generate configs based on the parameters required

Output:
appropriate yaml files in config directory
'''

import sys, os
import rospkg
import random as rn
import tpbp_functions as fn
import glob 
import yaml

if len(sys.argv[1:]) == 0:
    print('Specify Algorithm and corresponding parameters')   
else:
    alg_name = sys.argv[1]
    parameters = sys.argv[2:]
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    file_str = 'prior_pat_configs'
    dirname += '/config/'

    os.mkdir(dirname + alg_name)
    conf_files = glob.glob(dirname + file_str + '/{}*'.format(file_str))

    i = 0
    for c in conf_files:
        print(c)
        with open(c, 'r') as f:
            p = yaml.safe_load(f)
        p['algo_name'] = alg_name
        p['parameters'] = parameters
        with open(dirname + alg_name + '/{}_{}.yaml'.format(alg_name, i), 'w') as f:
            cur = yaml.dump(p, f)
        i += 1
    