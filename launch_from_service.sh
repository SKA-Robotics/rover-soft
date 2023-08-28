#!/bin/bash

source "/home/rover/.bashrc"

#TERM=xterm-256color tmux new-session -d -s ros -n roscore "bash -l -i -c \"roscore\"" 
#sleep 10 
#TERM=xterm-256color tmux new-window -d -t ros: -n nodes "bash -l -i -c \"source /home/jetson/rover-soft/devel/setup.bash; rosrun rosmon_core rosmon sirius_description sirius2.launch\""

TERM=xterm-256color tmux new-session -d -s ros -n nodes "bash -l -i -c \"source /home/rover/rover-soft/devel/setup.bash; roslaunch sirius_description sirius2.launch\""
TERM=xterm-256color tmux new-window -d -t ros: -n web_server "source /home/rover/.nvm/nvm.sh; cd /home/rover/robot-web-interface/server; npm run start; bash -i"
