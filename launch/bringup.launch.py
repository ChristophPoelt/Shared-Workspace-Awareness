from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node



def generate_launch_description():
    demo = LaunchConfiguration("demo")
    spawn_gripper_controller = LaunchConfiguration("spawn_gripper_controller")
    controller_manager = LaunchConfiguration("controller_manager")
    is_demo = IfCondition(PythonExpression(["'", demo, "' == 'true'"]))
    is_real = IfCondition(PythonExpression(["'", demo, "' == 'false'"]))

    # demo AND spawn_gripper_controller
    spawn_gripper = IfCondition(
        PythonExpression([
            "'", demo, "' == 'true' and '",
            spawn_gripper_controller, "' == 'true'"
        ])
    )

    transcriber = TimerAction(
        period=5.0,  # delay to let ROS 2 and other nodes come up first
        actions=[
            Node(
                package="meeseeks",
                executable="transcriber",
                name="transcriber",
                output="screen",
                respawn=True,           # restart if it crashes
                respawn_delay=3.0,      # wait 3s before restarting
            )
        ]
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
            description="Demo-only: spawn gripper_controller (JointGroupPositionController) via spawner",
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
        ),

        # --- Robot initialization: real only (if needed) ---
        Node(
            package="meeseeks",
            executable="robot_initialization",
            name="robot_initialization",
            output="screen",
            condition=is_real,
        ),

        # --- Dummy carriage only in demo (because sim doesn't publish it) ---
        Node(
            package="meeseeks",
            executable="demo_dummy_carriage",
            name="demo_dummy_carriage",
            output="screen",
            condition=is_demo,
        ),

        # --- Spawn gripper controller in demo (delayed to avoid controller_manager race) ---
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "gripper_controller",
                        "--controller-manager", controller_manager,
                        "--activate",
                    ],
                    output="screen",
                    condition=spawn_gripper,
                )
            ],
        ),

        # --- Always run your logic nodes ---
        Node(
            package="meeseeks",
            executable="main_logic",
            name="main_logic",
            output="screen",
            parameters=[{"demo_mode": demo}],  # optional if you read this param in code
        ),
        Node(
            package="meeseeks",
            executable="pointing_to_target_logic",
            name="pointing_to_target",
            output="screen",
        ),
        #transcriber,
    ])
