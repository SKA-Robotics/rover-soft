#!/bin/bash

slcand -o -s3 -t hw -S 115200 /dev/ttyACM0
ip link set up slcan0

sudo -i -u rover bash << EOF

source "/home/rover/.bashrc"

TERM=xterm-256color tmux new-session -d -s ros -n nodes "bash -l -i -c \"source /home/rover/rover-soft/devel/setup.bash; roslaunch sirius_description sirius2.launch\""
TERM=xterm-256color tmux new-window -d -t ros: -n web_server "source /home/rover/.nvm/nvm.sh; cd /home/rover/robot-web-interface/server; npm run start; bash -i"
TERM=xterm-256color tmux new-window -d -t ros: -n cameras "sudo tee /sys/module/uvcvideo/parameters/bandwidth_cap; docker compose run gstreamer-cameras; bash -i"
TERM=xterm-256color tmux new-window -d -t ros: -n power_readings "source /home/rover/rover-soft/devel/setup.bash; until rostopic list ; do sleep 1; done; rostopic echo -p /power_status | tee /home/rover/power_readings/$(date +"%Y-%m-%dT%H:%M:%S%z"); bash -i"

EOF
