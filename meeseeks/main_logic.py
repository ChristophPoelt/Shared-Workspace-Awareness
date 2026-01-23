#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from std_srvs.srv import Trigger

from meeseeks.targetSelection import selectNewTarget
from meeseeks.globalVariables import currentTargetGlobal

class GestureController:

    # wrapper around RobotGestures Trigger services.
    def __init__(self, node: Node, cb_group: ReentrantCallbackGroup):
        self._node = node
        self._cli_target_selected = node.create_client(
            Trigger, "/gesture/target_selected", callback_group=cb_group
        )
        self._cli_pause = node.create_client(
            Trigger, "/gesture/pause", callback_group=cb_group
        )
        self._cli_resume = node.create_client(
            Trigger, "/gesture/resume", callback_group=cb_group
        )
        self._cli_abort = node.create_client(
            Trigger, "/gesture/abort", callback_group=cb_group
        )

    def _call_async(self, client, label: str) -> None:
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().warn(f"Service not available: {label}")
                return

        fut = client.call_async(Trigger.Request())

        def _done_cb(f):
            try:
                resp = f.result()
                if resp and resp.success:
                    self._node.get_logger().info(f"{label}: OK ({resp.message})")
                elif resp:
                    self._node.get_logger().warn(f"{label}: FAIL ({resp.message})")
                else:
                    self._node.get_logger().warn(f"{label}: no response")
            except Exception as e:
                self._node.get_logger().error(f"{label}: exception: {e}")

        fut.add_done_callback(_done_cb)

    # gesture call
    def target_selected(self) -> None:
        self._call_async(self._cli_target_selected, "/gesture/target_selected")

    # Used by Christoph section (optional)
    def pause(self) -> None:
        self._call_async(self._cli_pause, "/gesture/pause")

    def resume(self) -> None:
        self._call_async(self._cli_resume, "/gesture/resume")

    def abort(self) -> None:
        self._call_async(self._cli_abort, "/gesture/abort")


class MainLogic(Node):
    def __init__(self):
        super().__init__("main_logic")

        self.cb_group = ReentrantCallbackGroup()

        # Service client for robot initialization (Franzi)
        self.robot_init_client = self.create_client(
            Trigger,
            "/robot/initialize",
            callback_group=self.cb_group,
        )

        # TODO: [Christoph] confirm voice topic content & allowed commands
        self.voice_sub = self.create_subscription(
            String,
            "/voice_commands",
            self._on_voice_command,
            10,
            callback_group=self.cb_group,
        )

        # gesture client wrapper
        self.gesture = GestureController(self, self.cb_group)

        # ----- Init sequence -----
        # done [Franzi] bring robot into initial position (drive to 0.0 + reset arm pose)
        self._initial_pose()

        # select initial target
        self._select_new_target_safe(initial=True)

        # target selected gesture
        self.gesture.target_selected()

        # TODO: [Franzi] target indication gesture (pointing)
        self._target_indication()

        # Main loop timer
        self.control_timer = self.create_timer(
            0.1, self.main_control_loop, callback_group=self.cb_group
        )

    # -------------------------
    # Christoph: Voice commands
    # -------------------------
    def _on_voice_command(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        cmd = raw.lower().replace(" ", "")
        if not cmd:
            return

        if cmd == "abort":
            self.abort_command_logic()
        elif cmd == "pause":
            self.pause_command_logic()  # must include gesture
        elif cmd in ("whereareyougoing", "where_are_you_going"):
            self.where_are_you_going_logic()
        elif cmd == "continue":
            self.continue_logic()
        else:
            self.get_logger().warn(f"Unknown voice command: '{raw}'")

    # TODO: [Christoph]
    def abort_command_logic(self) -> None:
        self.get_logger().info("TODO[Christoph]: abort_command_logic()")
        self.gesture.abort()

    # TODO: [Christoph] (needs to include doing the gesture)
    def pause_command_logic(self) -> None:
        self.get_logger().info("TODO[Christoph]: pause_command_logic()")
        self.gesture.pause()

    # TODO: [Christoph]
    def where_are_you_going_logic(self) -> None:
        self.get_logger().info("TODO[Christoph]: where_are_you_going_logic()")
        self.get_logger().info(f"Current target: {currentTargetGlobal}")

    # TODO: [Christoph]
    def continue_logic(self) -> None:
        self.get_logger().info("TODO[Christoph]: continue_logic()")
        self.gesture.resume()

    # -------------------------
    # Main control loop
    # -------------------------
    def main_control_loop(self) -> None:
        # TODO: [Team] logic if target is reached -> define later
        if not self._is_target_reached():
            return

        # select next target
        self._select_new_target_safe(initial=False)

        # target selected gesture
        self.gesture.target_selected()

        # TODO: [Franzi] point towards new target
        self._target_indication()

    # TODO: [Team]
    def _is_target_reached(self) -> bool:
        return False

    # -------------------------
    # Helpers
    # -------------------------
    def _select_new_target_safe(self, initial: bool) -> None:
        try:
            if initial:
                selectNewTarget()
            else:
                selectNewTarget(currentTargetGlobal)
        except TypeError:
            # fallback if signature differs
            try:
                selectNewTarget(currentTargetGlobal)
            except Exception as e:
                self.get_logger().error(f"selectNewTarget failed: {e}")
        except Exception as e:
            self.get_logger().error(f"selectNewTarget failed: {e}")

    # -------------------------
    # Franzi placeholders
    # -------------------------
    def _initial_pose(self) -> None:
        self.get_logger().info("Requesting robot initialization...")

        if not self.robot_init_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Robot initialization service not available")
            return

        future = self.robot_init_client.call_async(Trigger.Request())

        def _done_cb(f):
            try:
                resp = f.result()
                if resp.success:
                    self.get_logger().info("Robot initialized successfully")
                else:
                    self.get_logger().warn("Robot initialization failed")
            except Exception as e:
                self.get_logger().error(f"Initialization service error: {e}")

        future.add_done_callback(_done_cb)


    # No action needed here: pointing_to_target_logic node handles moving joint_1 toward the current target
    def _target_indication(self) -> None:
        self.get_logger().info(
            "Pointing gesture is handled by the separate pointing_to_target_logic node"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MainLogic()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()



#from targetSelection import selectNewTarget
#from globalVariables import currentTargetGlobal

#here we could handle the starting of all the nodes etc.

#init function that:
#subscribes to all needed nodes 
#starts all needed nodes
#-> Shiyi copies here already existing structure for the init function

#bring robot in inital position (drive to 0.0 and correct arm position)
#-> Franzi

#select an inital target
#selectNewTarget

#do the target selected gesture -> Shiyi

#do the target indication gesture (point towards target) ->  Franzi

#if loop that handles all the voice commands and then 
#calls the according python script

#pseudo code: -> Christoph
#if abort command was detected:
#   call abort_command_logic
#if pause command was detected:
#   call pause_command_logic (needs to include doing the gesture)
#if whereAreYouGoing is detected:
#   call whereAreYouGoing_logic
#if continue is detected:
#   call continue_logic

#logic if target is reached -> we will figure it our later

#select the next target
#selectNewTarget(currentTargetGlobal)