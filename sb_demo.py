"""
TODO: 
* require to call switch before reset, or call switch from constructor with default task (0?)
* adapt reset for all envs, or else give them **kwargs
"""

from stable_baselines3 import DQN
import gymnasium as gym ;
import numpy as np ;

from  line_following import LineFollowingEnvironment ;

from stable_baselines3.common.env_checker import check_env
import sys ;


print(sys.argv) ;
if len(sys.argv) < 2:
  print("1st cmd line param must be root dir!") ;
  sys.exit(0);

env = LineFollowingEnvironment(root_dir=sys.argv[1], 
        obs_per_sec_sim_time = 15, 
        fake_inputs="yes", 
        external_steering="yes", 
        max_steps_per_episode = 30, 
        task_list = "0,0") ;
env.switch(0) ;
check_env(env) ;

model = DQN("MlpPolicy", env, verbose=10)
model.learn(total_timesteps=100)

"""
vec_env = model.get_env()
obs = vec_env.reset()
for i in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = vec_env.step(action)
    vec_env.render()
    # VecEnv resets automatically
    # if done:
    #   obs = env.reset()
"""

env.close()
