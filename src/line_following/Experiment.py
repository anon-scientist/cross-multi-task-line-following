""" Experimental experiment using the new DQN learner
    TODO: create own argparser for RLAgent and other global params
"""
from datetime import datetime
from cl_experiment.parsing import Command_Line_Parser, Kwarg_Parser


# ICRL imports
from gazebo_sim.learner import DQNLearner
from gazebo_sim.agent import RLAgent ;

from line_following import LineFollowingEnvironment
#from line_following import ModifiedDQNLearner

if __name__== "__main__":

    print(f'Begin execution at: {datetime.now()}')

    cmd_line_args = Command_Line_Parser().parse_args() ;
    p = Kwarg_Parser(**cmd_line_args) ;
    p.add_argument("--obs_per_sec_sim_time", type=int, required=True, default=15) ;
    p.add_argument("--benchmark", type=str, required=True) ;
    p.add_argument("--algorithm", type=str, required=True) ;
    config, unparsed = p.parse_known_args() ;
    
    # instantiate environment
    # compute nsec delay between two observations
    # complicated by the fact that gazebo computes step durations only to msec precision
    # so a frame rate of 30 means that the delay between two frames is 33msec, but not 33.3333 msec
    # so we have to round down if we want to work with nsec delays     
    hz = 30. ; # we have to know this, definedi n the robot sdf file, camera sensor plugin
    nsec_per_frame = int(1000./hz) * 1000000. ;
    nsec = nsec_per_frame * (hz / config.obs_per_sec_sim_time) ;
    print("Assumed time per frame: ", nsec) ;
    env = LineFollowingEnvironment(step_duration_nsec=nsec, **cmd_line_args)
    print("id=", env.get_input_dims()) ;

    
    if config.algorithm == "DQN":
      learner = DQNLearner(n_actions=env.action_space.n,
                                 obs_space=env.observation_space.shape,
                                 config=None, **cmd_line_args) ; 


    # instantiate agent
    agent = RLAgent(env, learner, **cmd_line_args)

    # execute experiment
    agent.go()
    print(f'Finish execution at: {datetime.now()}')
    agent.mop_up(); # Terminates debug thread so program can exit

