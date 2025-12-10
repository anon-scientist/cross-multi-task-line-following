#!/bin/bash
# starts a PYthon script that demonstrates use of the environment object, without RL framework
# Typically, execute in MLF main folder with 
# source env_only.bash $(pwd)/..

PROCESSES=(
    "gz.*sim"
    "different_forms.sdf"
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


# *------------ COMMON DEFINITIONS ----------------------

PROJECT_DIR="line-following-scenario"
if [ "$#" == "1" ] ; then
SRC_PATH=${1} ;
else
SRC_PATH="./../" ;
fi

WORLD_DIR="/simulation/gazebo/skript_world/world"
ROOT_PATH="${SRC_PATH}/${PROJECT_DIR}"
WORLD_PATH="${SRC_PATH}/${PROJECT_DIR}/${WORLD_DIR}"
OBJ_DATA="${WORLD_PATH}/obj_data.txt"
ICRL_PATH="${SRC_PATH}/icrl"
# *-------------------------------------------------------

# PYTHONPATH - PYTHONPATH - PYTHONPATH --------------------------------
export PYTHONPATH=$PYTHONPATH:${ROOT_PATH}/src
export PYTHONPATH=$PYTHONPATH:${SRC_PATH}/dcgmm/src
export PYTHONPATH=$PYTHONPATH:${SRC_PATH}/cl_suite/cl_experiment/src
export PYTHONPATH=$PYTHONPATH:${SRC_PATH}/cl_suite/ar/src
export PYTHONPATH=$PYTHONPATH:${SRC_PATH}/icrl/src
# *--------------------------------------------------------------------

# GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ
export GZ_VERSION="8"
export GZ_DISTRO="harmonic"
export GZ_IP="127.0.0.1"
export GZ_PARTITION="$(hostname)"
export GZ_TRANSPORT_RCVHWM=10
export GZ_TRANSPORT_SNDHWM=10
export GZ_TRANSPORT_LOG_SQL_PATH="~/.gz/"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${ROOT_PATH}/simulation/gazebo/models:${WORLD_PATH}:${ICRL_PATH}/models"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
# GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ - GZ

# start Gazebo ----------------------------------
# start gazebo without GUI
sim_options=" -v 4 -r -s --headless-rendering --render-engine ogre"
# start with GUI, comment out accordingly
#sim_options=" -v 4 -r "
gz sim ${sim_options} "${ROOT_PATH}/simulation/gazebo/colored_line_following.sdf"  &
# ---------------------------------------------

python3 sb_demo.py ${ROOT_PATH}

execute_watchout
