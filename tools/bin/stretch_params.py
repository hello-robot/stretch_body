#!/usr/bin/env python

import stretch_body.hello_utils as hello_utils
import importlib
import stretch_body.robot_params

"""
Print out the robot parameters to the command line, 
including where each parameter value originates from
"""
def build_rep(prefix, params, origin,rep):
    for k in params.keys():
        np = prefix + '.' + str(k)
        if type(params[k]) == dict:
            rep.update(build_rep(np, params[k], origin,rep))
        else:
            rep[np]=[params[k], origin]
    return rep

rep={}
user_origin='stretch_re1_user_params.yaml'
user_params = hello_utils.read_fleet_yaml(user_origin)

factory_origin=user_params['factory_params']
factory_params=hello_utils.read_fleet_yaml(factory_origin)
build_rep('param',factory_params,factory_origin,rep)

factory_origin='stretch_body.robot_params.factory_params'
build_rep('param',stretch_body.robot_params.factory_params,factory_origin,rep)

for outside_params_origin in user_params.get('params', []):
    outside_params=getattr(importlib.import_module(outside_params_origin),'params')
    build_rep('param', outside_params, outside_params_origin, rep)

build_rep('param',user_params,user_origin,rep)

print('#'*60+ ' Parameters for %s '%hello_utils.get_fleet_id()+'#'*60)
print ("{:<70} {:<70} {:<30}".format('Origin','Parameter','Value'))
print('-'*170)
for param in rep.keys():
    print("{:<70} {:<70} {:<30}".format(str(rep[param][1]), param, str(rep[param][0] )))

