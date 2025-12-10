# Multi-Task Line-Following (MLF), part of the CRoSS benchmark suite

This repository contains the Multi-Task Line-Following (MLF) benchmark from the CRoSS suite.

*This repository is provided exclusively for anonymous review purposes.
To preserve anonymity during the submission process, the repository is maintained in read-only mode and does not accept issues, pull requests, or discussions at this stage.
Upon acceptance of the paper, we will make the full public version of this project available under our official organization account.*

## Overview
The MLR benchmark evaluates continual reinforcement learning for a two-wheeled robot that controls its movement via a differential drive. Attached to the robot are 
a bumper sensor, a down-looking line camera and a lidar sensor. The robot can choose to activate one of 6 LEDs located on the robot chassis.

The action space contains 18 discrete actions: the 3 base actions (left/right/straight) combined with one of 6 LEDs to be switched on.

The benchmark consists of 150 tracks, each defined by a distinct 3D start position situated on a colored line whose boundary must be followed. 
Depending on the color of the line, the color of the surrounding ground area, and the distance from a wall, the robot must additionally turn on
a certain LED. 

Each task is composed of several tracks to be learned, and is usually configured such that only one track changes from task to task

## Files in root directory
* `robot_only.bash / robot_only.py`
Starts a demo demonstrating the standalone use of the robot, i.e., without environment or RL framework
* `env_only.bash / env_only.py`
Starts a demo demonstrating the standalone use of the environment, i.e., without the RL framework
* `main.bash`
Starts an experimental run using our own RL framework. Expects as a cmd line argument the root path where all repositories have been cloned to, typically use $(pwd)/..
* `create_tracks.py`
Re-creates tracks.txt according to the cmd line params, only use this if you know what you are doing. See also comments in the file itself!
* `create_tasks.py`
Re-creates tasks.txt according to the cmd line params, only use this if you know what you are doing. See also comments in the file itself!
* `tasks.txt`
* `track_defs.txt`

## Other files
* `simulation/gazebo/colored_line_following.sdf`
Main world file for Gazebo. Makes use of models defined in icrl/models and the local models subdirectory
* `src/line_following/LineFollowing.py`
Contains the gymnasium-like class LineFollowing for interaction with an arbitrary RL framework
* `src/line_following/Experiment.py`
Starts the main experimental loop using our own RL framework. Is called from main.bash with command line parameters.
This experiment is completely scriptable, i.e., defined by cmd line parameters.

## Cmd line parameters
src/different_forms/Experiment.py is called from main.bash with many cmd line params, some of these are:
```
        --benchmark                                         mydf                # leave untouched
        --debug_port                                        11001               # for external debugging via RLAGent class
        --seed                                              42                  # leave untouched
        --exp_id                                            DF-DQN              # determines logging directory
        --root_dir                                          "${ROOT_PATH}"      # should be absolute path of MPO rpository
        --obs_per_sec_sim_time                              15                  # how many observations per second of **simulated** time
        --task_list                                         1,1                 # can be 1,2,3,4 or 1-5,6,7-10 or similar
        --training_duration                                 5000                # training steps
        --evaluation_duration                               20                  # evaluation epsisodes with exploration turned off
        --training_duration_unit                            timesteps
        --evaluation_duration_unit                          episodes
        --max_steps_per_episode                             30                  # episode duration if not terminated before
        --start_task                                        0                   # load model weights .hd5 file before this task if > 0
        --eval_start_task                                   0                   # at what task do we start evaluation? Set >0 if taks 1 is, eg, a pure babbling task
        --exploration_start_task                            0                   # all tasks before this one are pure exploration. Usually 0.
        --training_duration_task_0                          5000                # specify different duration for task0 (if, eg, a babbling task)
        --fake_inputs                                       yes                 # MPO specific: use raw or preprocessed inputs (simplified setting)
        --external_steering                                 yes                 # MPO specific: use non-adaptive algo for steering (super-simplified setting together with fake_inputs "yes")
        --gamma                                             0.9                 # DQN discount factor
        --train_batch_size                                  100
        --algorithm                                         DQN                 # leave untouched
        --dqn_fc1_dims                                      128
        --dqn_fc2_dims                                      64
        --dqn_adam_lr                                       1e-4
        --dqn_dueling                                       no                  # obsolete
        --dqn_target_network                                yes                 # double dqn?
        --dqn_target_network_update_freq                    200                 # if double DQN: when to update target DNN
        --output_size                                       2                   # nr neurons in last DNN layer. Must be 2 for external_steering = "yes"
        --exploration                                       eps-greedy          # leave untouched
        --initial_epsilon                                   1.0                 # eps at start of a task
        --final_epsilon                                     0.2                 # lower bound for epsilon
        --epsilon_delta                                     0.00015             # linear epsilon decrease per step
        --eps_replay_factor                                 0.8                 # start value for epsilon if task > 0
        --replay_buffer                                     default             # leave untouched
        --capacity                                          1000                # replay buffer size
        --debug no                                                              # extra displays and  trigggering by TCP/IP
```


## Requirements
* Apptainer software
* icrl repository and cl_experiment repositories

## Installation and execution
Proceed with the following steps:
* Select a base path <path>
* chdir to <path> and clone the ICRL, cl_experiment and MLF (this one) repositories, see below for links
* if not done yet: generate the .SIF file with
```
singularity build icrl.sif icrl/icrl.def
```
* run a shell in the container:
```
singularity shell --nv icrl.sif
```
* chdir to the MLF directory and, e.g., start the main experiment file with the parameter piinting to <path>. A common way is to type
```
source main.bash $(pwd)/.. 
```

## HPC integration
The CroSS-benchmarks are explicitly designed to be executed in an HPC environment using, e.g., slurm workload manager.
A typical invocation would be done by **not** starting a singularity shell, but to execute main.bash directly, on a single node in this case:
```
chmod 777 main.bash
srun -N1 singularity exec --nv ../icrl.sif ./main.bash $(pwd)/.. 
```

## Related Repositories
* [**CRoSS** - Entry Repository](https://github.com/anon-scientist/continual-robotic-simulation-suite/)
* [ICRL - RL-Framework Repository](https://github.com/anon-scientist/cross-icrl/)
* [CL_Experiments - Utils Repository](https://github.com/anon-scientist/cl_experiment/)
