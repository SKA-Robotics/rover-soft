#!/bin/bash

source "/home/jetson/.bashrc"

TERM=xterm-256color tmux new-session -d -s ros -n roscore "bash -l -i -c \"roscore\"" 
sleep 10 
TERM=xterm-256color tmux new-window -d -t ros: -n nodes "bash -l -i -c \"source /home/jetson/rover-soft/devel/setup.bash; rosrun rosmon_core rosmon sirius_description sirius2.launch\""
