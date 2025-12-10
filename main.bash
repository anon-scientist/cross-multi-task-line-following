#!/bin/bash
# first parameter: base path
# Typically, start with
# source main.bash $(pwd)/.. 
#
PROCESSES=(
    "gz.*sim"
    "line_following.sdf"
    "colored_line_following.sdf"
    "gazebo_simulator"
    "Experiment.py"
    "ruby"
    "gz"
)

function print_message {
    echo ${3}
}

function print_info { print_message "BLUE"   "INFO" "${*}" ; }
function print_warn { print_message "YELLOW" "WARN" "${*}" ; }
function print_ok   { print_message "GREEN"  "OK"   "${*}" ; }
function print_err  { print_message "RED"    "ERR"  "${*}" ; }
function print_part { print_message "CYAN"   "PART" "${*}" ; }
function print_unk  { print_message "PURPLE" "UNK"  "${*}" ; }

function check_process {
    pgrep -f "${1}"
}

function eval_state {
    local state=$?

    if (( $state == 0 ))
        then print_ok "success ${1}"
        else print_err "failed ${1}"
    fi
    return $state
}

function kill_process {
    pkill -9 -f "${1}"
}

function execute_check {
    print_info "check process ${entry}"
    eval_state $(check_process "${entry}")
}

function execute_kill {
    print_info "try to kill ${entry}"
    eval_state $(kill_process "${entry}")
}

function execute_watchout {
    print_info "watchout for possible zombies"
    for entry in ${PROCESSES[@]}
    do
        execute_check &&
        execute_kill
    done
}

function execute_state {
    state=$?
    if (( $state == 0 ))
        then print_ok "success (${1})"
        else print_err "failed (${1})"
    fi
    return $state
}

# *------------ COMMON DEFINITIONS ----------------------
EXP_ID="LF"
PROJECT_DIR="line-following-scenario"
if [ "$#" == "1" ] ; then
SRC_PATH=${1} ;
else
SRC_PATH="./../" ;
fi
ROOT_PATH="${SRC_PATH}/${PROJECT_DIR}"
# *-------------------------------------------------------

# PYTHONPATH - PYTHONPATH - PYTHONPATH --------------------------------
export PYTHONPATH=$PYTHONPATH:${ROOT_PATH}/src
export PYTHONPATH=$PYTHONPATH:${SRC_PATH}/icrl/src
export PYTHONPATH=$PYTHONPATH:${SRC_PATH}/cl_experiment/src
#export PYTHONPATH=$PYTHONPATH:${SRC_PATH}/dcgmm/src
# *--------------------------------------------------------------------

# GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ
export GZ_VERSION="8"
export GZ_DISTRO="harmonic"
export GZ_IP="127.0.0.1"
export GZ_PARTITION="$(hostname)"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${ROOT_PATH}/simulation/gazebo/models:${SRC_PATH}/icrl/models"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
# GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ

# kill zombies
execute_watchout

# start gazebo
# start with GUI
#sim_options="-v 4 -r --render-engine ogre2";
# start without gui
sim_options="-v 4 -r -s --headless-rendering --render-engine ogre";

gz sim ${sim_options} "${ROOT_PATH}/simulation/gazebo/colored_line_following.sdf"  &

# start RL
echo  Executing line_following.Experiment &

# +++
python3 -m line_following.Experiment                                                                                        \
        --debug_port 11001                                                                                                  \
        --exp_id                                            "${EXP_ID}"                                                     \
        --benchmark                                         colored-lf                                                      \
        --fake_inputs                                       yes \
        --external_steering                                 yes \
        --obs_per_sec_sim_time                              15                                                              \
        --root_dir                                          "${ROOT_PATH}"                                                  \
        --start_task                                        0                                                               \
        --task_list                                         1,2 \
        --eval_start_task                                   0                                                               \
        --max_steps_per_episode                             30                                                              \
        --training_duration_unit                            timesteps                                                       \
        --evaluation_duration_unit                          episodes                                                        \
        --training_duration                                 5000                                                            \
        --training_duration_task_0                          10000                                                           \
        --evaluation_duration                               5                                                               \
        --train_batch_size                                  32                                                              \
        --gamma                                             0.8                                                             \
        --algorithm                                         DQN                                                             \
        --dqn_dueling                                       no                                                              \
        --dqn_target_network                                no                                                              \
        --dqn_target_network_update_freq                    100                                                             \
        --dqn_adam_lr                                       1e-3                                                            \
        --exploration                                       eps-greedy                                                      \
        --initial_epsilon                                   1.0                                                             \
        --epsilon_delta                                     0.00015                                                          \
        --final_epsilon                                     0.2                                                           \
        --replay_buffer                                     default                                                         \
        --eps_replay_factor                                 0.5                                                             \
        --eps_reset                                         0.5                                                             \
        --capacity                                          10000                                                           \
        --sequence_length                                   3                                                               \
        --seed                                              42                                                              \
        --debug                                             no                                                              \
        --verbose                                           yes                                                             \
        ; execute_state "Experiment"
# ---

echo DONE

# kill zombies
execute_watchout
