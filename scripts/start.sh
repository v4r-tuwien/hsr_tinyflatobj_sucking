#!/bin/bash

SESSION=Small_Object_Pipeline

tmux new -d -s $SESSION
tmux set -g mouse on
tmux new-window -t $SESSION:1 

# start handover and table_plane server
tmux select-window -t $SESSION:1
tmux split-window -h
tmux send-keys "rosrun handover handover_server.py" C-m
tmux select-pane -t 0
tmux send-keys "roslaunch table_plane_extractor table_plane_extractor.launch" C-m
tmux rename-window 'Table_Plane_Extractor and Handover Server'

# start find_object, arm_movement server and user_input shell
tmux select-window -t $SESSION:0
tmux split-window -h
tmux split-window -v
tmux send-keys "rosrun hsr_small_objects find_object_action_server.py" C-m
tmux select-pane -t 1
tmux send-keys "rosrun hsr_small_objects arm_movement_action_server.py" C-m
tmux select-pane -t 0
tmux send-keys "rosrun hsr_small_objects user_input.py" C-m
tmux rename-window 'User Interface'

# Attach to termial
tmux attach-session -t $SESSION
