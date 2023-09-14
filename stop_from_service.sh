#!/bin/bash

sudo -i -u rover bash << EOF

source "/home/rover/.bashrc"

tmux kill-session -t ros

EOF
