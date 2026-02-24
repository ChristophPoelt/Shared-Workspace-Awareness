from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    demo = LaunchConfiguration("demo")
    spawn_gripper_controller = LaunchConfiguration("spawn_gripper_controller")
    controller_manager = LaunchConfiguration("controller_manager")
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
                "for attempt in 1 2 3; do "
                "  echo \"[bringup] gripper_controller spawner attempt ${attempt}/3\"; "
                "  if ros2 run controller_manager spawner gripper_controller "
                "--controller-manager ",
                controller_manager,
                " --activate; then "
                "    echo '[bringup] gripper_controller spawn confirmed'; "
                "    exit 0; "
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
        # --- Always run gestures (so you can test them) ---
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
            }],
        ),

        # --- Robot initialization: both only (if needed) ---
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
            }],
        ),

        # --- Spawn gripper controller in demo (delayed + retried to avoid controller_manager race) ---
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="[bringup] Demo: launching gripper_controller spawner"),
                gripper_spawner_with_retry,
            ],
        ),

        # --- Always run your logic nodes ---
        TimerAction(
            period=6.5,
            actions=[
                Node(
                    package="meeseeks",
                    executable="main_logic",
                    name="main_logic",
                    output="screen",
                    parameters=[{"demo_mode": demo}],
                ),
            ],
        ),
        Node(
            package="meeseeks",
            executable="pointing_to_target_logic",
            name="pointing_to_target",
            output="screen",
            parameters=[{
                "max_joint1_speed_rad_s": 0.3,
                "min_move_duration_s": 0.5,
                "max_move_duration_s": 5.0,
                "command_period_s": 0.2,
                "joint1_deadband_rad": 0.01,
                "fixed_rail_pos": 0.0,
            }],
        ),
        # Start adapter immediately in demo so action server is ready before logic triggers gestures.
        gripper_adapter,
        # Transcriber is intentionally NOT launched here.
        # Run it separately when needed, or use voice_cli_publisher for manual typed commands.
    ])
