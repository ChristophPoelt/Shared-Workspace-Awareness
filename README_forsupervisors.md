# Shared-Workspace-Awareness

# Install Torch separately (recommended because wheels are platform-specific)
pip install torch

# Install the remaining transcriber dependencies
pip install -r requirements-voice.txt

# Verify imports work in the venv
python -c "import torch, whisper, sounddevice; from silero_vad import load_silero_vad; print('ok')"

# Build with this environment active so the ROS console script points at this Python
colcon build --packages-select meeseeks
source install/setup.bash

# Optional debug check:
# Voice state control
ros2 topic echo /voice_commands
# Robot control state:
ros2 topic echo /robot_control_state
```
Notes:
- If `openai-whisper` later reports missing `ffmpeg`, install `ffmpeg` on your system. Install libportaudio2, ffmpeg dependency.  
.

## Build project
colcon build --packages-select meeseeks [--symlink-install]
source install/setup.bash

## Launch with the real robot
ros2 launch meeseeks bringup.launch.py demo:=false
## Normal demo  simulation launch
ros2 launch meeseeks bringup.launch.py demo:=true
## If your already handles gripper controller (avoid duplicates)
ros2 launch meeseeks bringup.launch.py demo:=true spawn_gripper_controller:=false
## If your controller_manager is namespaced (rare, but possible)
ros2 launch meeseeks bringup.launch.py demo:=true controller_manager:=/my_ns/controller_manager

## Run transcriber or voice_cli_publisher in separate Terminal
source install/setup.bash

ros2 run meeseeks transcriber
ros2 run meeseeks voice_cli_publisher



## Set spawner works manually
ros2 run controller_manager spawner gripper_controller --controller-manager /controller_manager --activate
## check if they really spawned
ros2 control list_controllers | grep -i gripper


