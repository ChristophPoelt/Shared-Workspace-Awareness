# Shared-Workspace-Awareness

## Launch with the real robot
ros2 launch meeseeks bringup.launch.py demo:=false
## Normal demo  simulation launch
ros2 launch meeseeks bringup.launch.py demo:=true
## If your already handles gripper controller (avoid duplicates)
ros2 launch meeseeks bringup.launch.py demo:=true spawn_gripper_controller:=false
## If your controller_manager is namespaced (rare, but possible)
ros2 launch meeseeks bringup.launch.py demo:=true controller_manager:=/my_ns/controller_manager