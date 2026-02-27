from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    demo = LaunchConfiguration("demo")
    spawn_gripper_controller = LaunchConfiguration("spawn_gripper_controller")
    controller_manager = LaunchConfiguration("controller_manager")
    demo_auto_ready = LaunchConfiguration("demo_auto_ready")
    demo_auto_armed = LaunchConfiguration("demo_auto_armed")
    demo_only = IfCondition(PythonExpression(["'", demo, "' == 'true'"]))
    # demo AND spawn_gripper_controller
    spawn_gripper = IfCondition(
        PythonExpression([
            "'", demo, "' == 'true' and '",
            spawn_gripper_controller, "' == 'true'"
        ])
    )

    gripper_adapter = Node(
        package="meeseeks",
        executable="gripper_action_adapter",
        name="gripper_action_adapter",
        output="screen",
        condition=demo_only,
        parameters=[{
            "min_pos": 0.0,
            "max_pos": 0.8,
            "settle_time_sec": 0.2,
        }],
    )

    gripper_spawner_with_retry = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            [
                "echo '[bringup] Spawning gripper_controller (demo mode)'; "
                "get_gripper_state() { "
                "  timeout 4s ros2 control list_controllers --controller-manager ",
                controller_manager,
                " 2>/dev/null | awk '$1==\"gripper_controller\" {print tolower($NF); found=1} END {if (!found) print \"\"}'; "
                "}; "
                "dump_controllers() { "
                "  echo '[bringup] list_controllers:'; "
                "  timeout 4s ros2 control list_controllers --controller-manager ",
                controller_manager,
                " || echo '[bringup] WARN: list_controllers unavailable/timed out'; "
                "}; "
                "for attempt in 1 2 3; do "
                "  echo \"[bringup] gripper_controller spawner attempt ${attempt}/3\"; "
                "  dump_controllers; "
                "  state=$(get_gripper_state); "
                "  echo \"[bringup] gripper_controller state(before)=${state:-missing}\"; "
                "  if [ \"$state\" = active ]; then "
                "    echo '[bringup] gripper_controller already active; skipping spawner'; "
                "    exit 0; "
                "  fi; "
                "  if [ -z \"$state\" ]; then "
                "    echo '[bringup] gripper_controller missing -> load only'; "
                "    ros2 run controller_manager spawner gripper_controller "
                "--controller-manager ",
                controller_manager,
                " --load-only || true; "
                "    state=$(get_gripper_state); "
                "    echo \"[bringup] gripper_controller state(after load)=${state:-missing}\"; "
                "  fi; "
                "  case \"$state\" in "
                "    active) "
                "      echo '[bringup] gripper_controller spawn confirmed'; "
                "      exit 0 ;; "
                "    inactive) "
                "      echo '[bringup] activating existing inactive gripper_controller'; "
                "      ros2 control set_controller_state gripper_controller active --controller-manager ",
                controller_manager,
                " || true ;; "
                "    unconfigured) "
                "      echo '[bringup] configuring then activating gripper_controller'; "
                "      ros2 control set_controller_state gripper_controller inactive --controller-manager ",
                controller_manager,
                " || true; "
                "      ros2 control set_controller_state gripper_controller active --controller-manager ",
                controller_manager,
                " || true ;; "
                "    finalized|error) "
                "      echo \"[bringup] ERROR: gripper_controller in terminal state=${state}; not retrying configure/activate\"; "
                "      dump_controllers; "
                "      exit 1 ;; "
                "    *) "
                "      echo \"[bringup] gripper_controller state=${state:-missing}; will retry\" ;; "
                "  esac; "
                "  dump_controllers; "
                "  state=$(get_gripper_state); "
                "  echo \"[bringup] gripper_controller state(after)=${state:-missing}\"; "
                "  if [ \"$state\" = active ]; then "
                "    echo '[bringup] gripper_controller spawn confirmed'; "
                "    exit 0; "
                "  fi; "
                "  if [ \"$state\" = finalized ] || [ \"$state\" = error ]; then "
                "    echo \"[bringup] ERROR: gripper_controller terminal state after attempt: $state\"; "
                "    exit 1; "
                "  fi; "
                "  echo '[bringup] gripper_controller spawner retrying in 2s'; "
                "  sleep 2; "
                "done; "
                "echo '[bringup] ERROR: failed to spawn gripper_controller after retries'; "
                "exit 1"
            ],
        ],
        output="screen",
        condition=spawn_gripper,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "demo",
            default_value="false",
            description="true: demo/sim mode. false: real system",
        ),
        DeclareLaunchArgument(
            "spawn_gripper_controller",
            default_value="true",
            description="Demo-only: spawn gripper_controller via spawner (set false if sim already loads it)",
        ),
        DeclareLaunchArgument(
            "controller_manager",
            default_value="/controller_manager",
            description="Controller manager node name (change if namespaced)",
        ),
        DeclareLaunchArgument(
            "demo_auto_ready",
            default_value="false",
            description="Pointing-node standalone/demo override: treat missing /robot_control_state as ready after grace period",
        ),
        DeclareLaunchArgument(
            "demo_auto_armed",
            default_value="false",
            description="Pointing-node standalone/demo override: treat missing /arm_armed as true after grace period",
        ),

        Node(
            package="meeseeks",
            executable="robot_gestures",
            name="robot_gestures",
            output="screen",
            parameters=[{
                "backend": PythonExpression([
                    "'action' if '", demo, "' == 'true' else 'auto'"
                ]),
                "pause_s": 1.0,
                "follow_control_state": True,
            }],
        ),

        Node(
            package="meeseeks",
            executable="robot_initialization",
            name="robot_initialization",
            output="screen",
            parameters=[{
                "max_joint_speed_rad_s": 0.3,
                "min_move_duration_s": 2.0,
                "max_move_duration_s": 12.0,
                "fallback_move_duration_s": 8.0,
                "joint_state_wait_timeout_s": 1.0,
                "publish_gating_topics": False,
            }],
        ),

        # Spawn gripper controller in demo
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="[bringup] Demo: launching gripper_controller spawner"),
                gripper_spawner_with_retry,
            ],
        ),

        TimerAction(
            period=6.5,
            actions=[
                Node(
                    package="meeseeks",
                    executable="main_logic",
                    name="main_logic",
                    output="screen",
                    parameters=[{
                        "demo_mode": demo,
                    }],
                ),
            ],
        ),
        Node(
            package="meeseeks",
            executable="pointing_to_target_logic",
            name="pointing_to_target",
            output="screen",
            parameters=[{
                "command_period_s": 0.2,
                "joint1_deadband_rad": 0.01,
                "fixed_rail_pos": 0.0,
                "moveit_group_name": "manipulator",
                "moveit_action_name": "/move_action",
                "demo_auto_ready": demo_auto_ready,
                "demo_auto_armed": demo_auto_armed,
            }],
        ),
        gripper_adapter,
    ])
