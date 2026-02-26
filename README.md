# Shared-Workspace-Awareness
export ROS_DOMAIN_ID=2
echo $ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch kortex_bringup gen3.launch.py robot_ip:=10.163.18.198 dof:=6 vision:=true gripper:=robotiq_2f_85 launch_rviz:=true
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=10.163.18.198 use_fake_hardware:=false



## Control pipeline (current architecture)
- `main_logic` is the canonical publisher for:
  - `/robot_control_state` (`std_msgs/String`)
  - `/arm_armed` (`std_msgs/Bool`)
  - `/selected_target` (`std_msgs/String`, when selecting targets)
- `pointing_to_target_logic` consumes those topics and performs MoveIt planning/execution.
- `robot_gestures` does not publish gating topics; it exposes gesture services and can optionally follow `/robot_control_state`.
- `robot_initialization` provides `/robot/initialize`; it can optionally publish gating topics for standalone/demo-only initialization (`publish_gating_topics:=true`), but this is disabled by default to avoid competing with `main_logic`.

All command/gating topics use CLI-friendly QoS (`RELIABLE + VOLATILE`, depth 1), so `ros2 topic pub` defaults should work without extra QoS flags.

## Minimal standalone pointing test (CLI only, no state publishers required)
Run the pointing node with demo overrides enabled (for local sim/dev only):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run meeseeks pointing_to_target_logic --ros-args \
  -p demo_auto_ready:=true \
  -p demo_auto_armed:=true \
  -p moveit_group_name:=manipulator \
  -p moveit_action_name:=/move_action
```

ros2 topic echo --once /robot_control_state
ros2 topic echo --once /arm_armed

Then publish a target (no QoS flags needed):

```bash
ros2 topic pub --once /selected_target std_msgs/msg/String "{data: position1}"
```

Pause/cancel test:

```bash
ros2 topic pub --once /robot_control_state std_msgs/msg/String "{data: paused}"
```

Resume:

```bash
ros2 topic pub --once /robot_control_state std_msgs/msg/String "{data: ready}"
```

## Full bringup in simulation (MoveIt + nodes)
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch meeseeks bringup.launch.py demo:=true
```

Optional demo fallback overrides for the pointing node (normally not needed when `main_logic` is running):

```bash
ros2 launch meeseeks bringup.launch.py demo:=true demo_auto_ready:=true demo_auto_armed:=true
```

Publish a target:

```bash
ros2 topic pub --once /selected_target std_msgs/msg/String "{data: position2}"
```

## Real robot bringup (remote server)
Safety notes:
- Keep `demo_auto_ready:=false` and `demo_auto_armed:=false` (defaults).
- Ensure only one `main_logic` instance is running (it is the canonical state publisher).
- Confirm controllers are available before initialization/motion.
- Verify the correct ROS domain is set on all machines.

Example:

```bash
export ROS_DOMAIN_ID=42   # replace with your deployment value
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch meeseeks bringup.launch.py demo:=false
```

Useful checks:

```bash
ros2 topic echo /robot_control_state
ros2 topic echo /arm_armed
ros2 control list_controllers
```

## Duplicate-node guard rails
- Nodes log `[INIT] duplicate node name detected ...` if multiple instances with the same ROS node name are present.
- This commonly happens when a launch file is already running and a second standalone `ros2 run ...` command is started.
- Wrapper processes and ROS node processes are different; rely on the ROS node name logs (`node=<name> pid=<pid>`) printed at startup.

## Voice transcriber setup (one-time per Python environment)
The `transcriber` is now launched as a normal ROS 2 node (`Node(package="meeseeks", executable="transcriber")`), so you do not need to edit `bringup.launch.py` or pass a custom `voice_python:=...` path.

Install the voice dependencies in the same Python environment you will use to build the package:

```bash
python3 -m venv .venv
source .venv/bin/activate

# Install Torch separately (recommended because wheels are platform-specific)
pip install torch

# Install the remaining transcriber dependencies
pip install -r requirements-voice.txt

# Activate venv
source .venv/bin/activate

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
- If `openai-whisper` later reports missing `ffmpeg`, install `ffmpeg` on your system.
- Rebuild `meeseeks` if you switch to a different Python environment and want the `transcriber` executable to use it.

## Launch variants
## Real robot
ros2 launch meeseeks bringup.launch.py demo:=false
## Demo simulation
ros2 launch meeseeks bringup.launch.py demo:=true
## If your sim already handles gripper controller (avoid duplicates)
ros2 launch meeseeks bringup.launch.py demo:=true spawn_gripper_controller:=false
## If your controller_manager is namespaced (rare, but possible)
ros2 launch meeseeks bringup.launch.py demo:=true controller_manager:=/my_ns/controller_manager

## Run transcriber or voice_cli_publisher in separate Terminal
colcon build --packages-select meeseeks --symlink-install
source install/setup.bash
source /opt/ros/jazzy/setup.bash

ros2 topic pub /elmo/id2/carriage/position/set std_msgs/msg/Float32 'data: 12.2'
ros2 topic pub /elmo/id2/lift/position/set std_msgs/msg/Float32 'data: 0.24'

ros2 run meeseeks transcriber
ros2 run meeseeks voice_cli_publisher
ros2 run meeseeks emergency_stop

## Set spawner works manually
ros2 run controller_manager spawner gripper_controller --controller-manager /controller_manager --activate
## check if they really spawned
ros2 control list_controllers | grep -i gripper

ros2 topic list | grep -E "robot_control_state|arm_armed"
ros2 topic echo --once /robot_control_state
ros2 topic echo --once /arm_armed
