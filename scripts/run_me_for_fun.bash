#!/bin/bash
#
# This script should be invoked by "ros2 run my_controller run_me_for_fun.bash" 
#
set -ue -o pipefail

PROG_DIR=$(dirname $(readlink -f "$0")) # where is the program located
EXEC_DIR=$(pwd -P)                      # where are we executing from
PROG_NAME=$(basename "$0")              # the program name without the path
PID=$$                                  # Process ID of this process,
                                        #   also the parent / head of this process group

# 1.) Get the ros package name
ROS_PACKAGE_NAME=$(basename $PROG_DIR)

echo
echo "ROS_PACKAGE_NAME = $ROS_PACKAGE_NAME"
echo "PROG_DIR = $PROG_DIR"
echo "EXEC_DIR = $EXEC_DIR"
echo "PROG_NAME = $PROG_NAME"
echo "PARAMS = $@"
echo

# 2.) some function
function add {
    echo "first param is $1"
    echo "2nd param is $2"
    echo "3rd param is $3"
    echo "sum is $(( $1 + $2 + $3 ))"
}
add 0 1 3

# 3.) signal handler
N=1
function signal_handler {
    echo "You have pressed control-c $N/3 times"
    if [[ $N == "3" ]]; then
        echo "Bye"
        # Kill all processes in this procees group
        # (including the group leader / parent itself)
        kill -s SIGKILL -$PID
    fi
    N=$(( N + 1 ))
}
trap signal_handler SIGINT 

# 4.) Child processees
sleep 1000&
sleep 1000&
sleep 1000&

# 5.) Wait for all children to exit
while wait || true; do
    echo "   'wait' has been killed -- or all children have exited"
    pgrep -P $PID > /dev/null   # fails if no more children
done
