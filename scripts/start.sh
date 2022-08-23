#!/bin/bash

SESSION=Small_Object_Pipeline

tmux new -d -s $SESSION
tmux set -g mouse on


## with map setting
tmux select-window -t $SESSION:0
tmux split-window -h
tmux split-window -v
tmux send-keys "rosrun hsr_small_objects find_object_action_server.py" C-m
tmux select-pane -t 1
tmux send-keys "rosrun hsr_small_objects pick_object_action_server.py" C-m
tmux select-pane -t 0
tmux send-keys "rosrun hsr_small_objects small_objects_statemachine.py" C-m
tmux rename-window 'statemachine'

# Attach to session
tmux attach-session -t $SESSION
