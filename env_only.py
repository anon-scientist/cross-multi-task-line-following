"""
CRoSS benchmark suite, MLF  benchmarkExample for standalone use of provided
environment, e.g., in your own RL framework.
Expects the root directory of the MLF repository as first argument!
"""

from  line_following import LineFollowingEnvironment ;
import sys ;

print(sys.argv) ;
if len(sys.argv) < 2:
  print("1st cmd line param must be root dir!") ;
  sys.exit(0);

env = LineFollowingEnvironment(root_dir=sys.argv[1]) ;

print("Define a task") ;
env.switch(0) ;
print("start an episode") ;
env.reset() ;
for i in range(0,10):
  print("execute action 0 and obtain results") ;
  action_index = 0 ;
  obs, reward, terminated, truncated, info = env.step(action_index) ;
  print("Reward=", reward) ;
