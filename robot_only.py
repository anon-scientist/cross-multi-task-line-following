"""
CRoSS benchmark suite, MLF  benchmarkExample for standalone use of provided
robot manager, without environment or RL framework
"""

import sys, numpy as np ;
from line_following import define_robot_actions, ThreePiWithLampManager ;
from gz.msgs10.color_pb2 import Color

world_name = 'line_following_world' ;
robot_name = '3pi_robot_with_line_cam' ;

actions = define_robot_actions(world_name, robot_name) ;

env_config = {"observation_shape":[2,50,3],"tasks":None,"actions":actions,"robot_name":robot_name,"vehicle_prefix":'/vehicle',
                                       "lidar":'/vehicle/lidar',"world_name":"/world/" + world_name,"camera_topic":'/vehicle/camera' } ;

manager = ThreePiWithLampManager(env_config) ;

for i in range(0,10):
  manager.gz_perform_action(actions[0]) ;

