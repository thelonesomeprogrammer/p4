#!/bin/bash

SESSION="ros_dashboard"

# Create a new detached tmux session
tmux new-session -d -s $SESSION -n vicon_bridge

# Window 1: vicon bridge
tmux send-keys -t $SESSION:0 'source /opt/ros/jazzy/setup.bash' C-m
tmux send-keys -t $SESSION:0 'source install/setup.bash' C-m
tmux send-keys -t $SESSION:0 'ros2 launch launch/all_segments.launch.py' C-m

# Window 2: ground station
tmux new-window -t $SESSION:1 -n ground_station
tmux send-keys -t $SESSION:1 'source /opt/ros/jazzy/setup.bash' C-m
tmux send-keys -t $SESSION:1 'source install/setup.bash' C-m
tmux send-keys -t $SESSION:1 'source /venv/bin/activate' C-m
tmux send-keys -t $SESSION:1 'python src/vicon-crazy/vicon_crazy/vicon_stream.py' C-m

# Window 3: web api
tmux new-window -t $SESSION:2 -n web_api
tmux send-keys -t $SESSION:2 'source /opt/ros/jazzy/setup.bash' C-m
tmux send-keys -t $SESSION:2 'source install/setup.bash' C-m
tmux send-keys -t $SESSION:2 'ros2 run cf_rust_gui web' C-m

# Window 4: bag recording
tmux new-window -t $SESSION:3 -n bag
tmux send-keys -t $SESSION:3 'source /opt/ros/jazzy/setup.bash' C-m
tmux send-keys -t $SESSION:3 'ros2 bag record --topics /cf_appchannel /vicon/Group466CF/Group466CF/pose /cf_command /cf_logpos /cf_controll -o bag1' C-m

# Window 5: bash shell
tmux new-window -t $SESSION:4 -n shell

# Attach to the session
tmux attach-session -t $SESSION

