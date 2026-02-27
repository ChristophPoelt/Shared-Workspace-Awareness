# Shared-Workspace-Awareness

## Environment Setup for real robot
export ROS_DOMAIN_ID=2
echo $ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch kortex_bringup gen3.launch.py robot_ip:=10.163.18.198 dof:=6 vision:=true gripper:=robotiq_2f_85 launch_rviz:=true
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=10.163.18.198 use_fake_hardware:=false

## Voice transcriber setup (one-time per Python environment)
The `transcriber` is launched as a normal ROS 2 node

### Install the voice dependencies in the same Python environment

python3 -m venv .venv
source .venv/bin/activate

### Install Torch separately
pip install torch

### Install the remaining transcriber dependencies
pip install -r requirements-voice.txt

### Activate venv
source .venv/bin/activate

### Verify imports work in the venv
python -c "import torch, whisper, sounddevice; from silero_vad import load_silero_vad; print('ok')"

### Build with this environment active so the ROS console script points at this Python
colcon build --packages-select meeseeks --symlink-install
source install/setup.bash
ros2 run meeseeks transcriber

Notes:
- If `openai-whisper` later reports missing `ffmpeg`, install `ffmpeg` on your system.
- Rebuild `meeseeks` if you switch to a different Python environment and want the `transcriber` executable to use it.

## Launch variants
### Real robot
ros2 launch meeseeks bringup.launch.py demo:=false

### Demo simulation
ros2 launch meeseeks bringup.launch.py demo:=true

---

## Control pipeline (current architecture)
- `main_logic` is the canonical publisher for:
  - `/robot_control_state`
  - `/arm_armed`
  - `/selected_target`
- `pointing_to_target_logic` consumes those topics and performs MoveIt planning/execution.
- `robot_gestures` does not publish gating topics; it exposes gesture services and can optionally follow `/robot_control_state`.
- `robot_initialization` provides `/robot/initialize`; it can optionally publish gating topics for standalone/demo-only initialization (`publish_gating_topics:=true`), but this is disabled by default to avoid competing with `main_logic`.

All command/gating topics use CLI-friendly QoS (`RELIABLE + VOLATILE`, depth 1), so `ros2 topic pub` defaults should work without extra QoS flags.

---

### Optional debug check:
### Voice state control
ros2 topic echo /voice_commands

### Robot control state:
ros2 topic echo /robot_control_state

