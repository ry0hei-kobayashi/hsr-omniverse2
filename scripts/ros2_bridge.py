# this code has been adapted from
#  https://github.com/Toni-SM/semu.robotics.ros2_bridge/blob/main/src/semu.robotics.ros2_bridge/semu/robotics/ros2_bridge/ros2_bridge.py
# the code is licensed under the MIT license
#  https://github.com/Toni-SM/semu.robotics.ros2_bridge/blob/main/LICENSE

from typing import List, Any

import time
import json
import asyncio
import threading

import omni
import carb
import omni.kit
from pxr import Usd, Gf, PhysxSchema
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.stage import get_stage_units

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from control_msgs.msg import JointTrajectoryControllerState


class RosController:
    def __init__(self, node: Node) -> None:
        """Base class for RosController

        :param node: ROS2 node
        :type node: Node
        """
        self._node = node        
        self.started = False
    
    def start(self) -> None:
        """Start the component
        """
        raise NotImplementedError

    def stop(self) -> None:
        """Stop the component
        """
        print("[Info][semu.robotics.ros2_bridge] RosController: stopping")
        self.started = False

    def step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        raise NotImplementedError


class RosControlFollowJointTrajectory(RosController):
    # Joints this controller manages. None = publish all articulation joints
    # (legacy behavior). Subclasses override with a list to scope the
    # /controller_state msg to the joints the consumer expects (pumas
    # arm_node segfaults when actual.positions oversized).
    controlled_joints = None

    def __init__(self,
                 node: Node,
                 _dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """FollowJointTrajectory interface
        
        :param node: The ROS node
        :type node: rclpy.node.Node
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(node)

        self.dci = _dci

        self._articulation = _dynamic_control.INVALID_HANDLE
        self._joints = {}

        self._action_server = None

        self._action_dt = 0.05
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_point_index = 1

        # feedback / result
        self._action_result_message = None
        self._action_feedback_message = FollowJointTrajectory.Feedback()

        # publishers for controller state: legacy /<c>/state and the
        # ros2_control standard /<c>/controller_state (pumas's arm_controller
        # / viewpoint_controller subscribe to the latter)
        self._state_pub = None
        self._controller_state_pub = None

        # topic-interface subscription for /<c>/joint_trajectory (mirrors
        # ros2_control's JointTrajectoryController)
        self._traj_sub = None

        # Topic msgs are received on the ROS executor thread; the actual goal
        # installation (which calls DCI for current joint state) is deferred
        # to step() on the main thread to avoid racing with the physics step.
        self._pending_topic_msg = None
        self._pending_lock = threading.Lock()

        # Pad `time_from_start == 0` to this many seconds so single-shot
        # trajectories (e.g. pumas initial-pose publish) have time to
        # physically reach the target before the trajectory ends.
        self._zero_time_pad_sec = 1.0

    def start(self, _articulation_path, _action_topic_name) -> None:
        """Start the action server
        """
        print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: starting")

        # get attributes and relationships
        self.articulation_path = _articulation_path
        self.action_topic_name = _action_topic_name

        # start action server
        self._action_server = ActionServer(self._node,
                                           FollowJointTrajectory,
                                           self.action_topic_name,
                                           execute_callback=self._on_execute,
                                           goal_callback=self._on_goal,
                                           cancel_callback=self._on_cancel,
                                           handle_accepted_callback=self._on_handle_accepted)
        print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: register action {}" \
            .format(self.action_topic_name))

        # Controller state publishers — both topic names so topic-only clients
        # (pumas) and legacy consumers see the same payload.
        state_topic = self.action_topic_name.replace('/follow_joint_trajectory', '/state')
        self._state_pub = self._node.create_publisher(
            JointTrajectoryControllerState,
            state_topic,
            10,
        )
        controller_state_topic = self.action_topic_name.replace('/follow_joint_trajectory', '/controller_state')
        self._controller_state_pub = self._node.create_publisher(
            JointTrajectoryControllerState,
            controller_state_topic,
            10,
        )

        # Topic interface that mirrors ros2_control's JointTrajectoryController
        traj_topic = self.action_topic_name.replace('/follow_joint_trajectory', '/joint_trajectory')
        self._traj_sub = self._node.create_subscription(
            JointTrajectory,
            traj_topic,
            self._on_trajectory_topic,
            10,
        )
        print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: subscribe topic {}" \
            .format(traj_topic))
        print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: publish state on {} and {}" \
            .format(state_topic, controller_state_topic))

        self.started = True

    def stop(self) -> None:
        """Stop the action server
        """
        super().stop()
        self._articulation = _dynamic_control.INVALID_HANDLE
        # destroy action server
        if self._action_server is not None:
            print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: destroy action server: {}" \
                .format(self.action_topic_name))
            # self._action_server.destroy()
            self._action_server = None
        # destroy topic subscription
        if self._traj_sub is not None:
            try:
                self._node.destroy_subscription(self._traj_sub)
            except Exception:
                pass
            self._traj_sub = None
        self._action_goal_handle = None
        self._action_goal = None

    def _duration_to_seconds(self, duration: Duration) -> float:
        """Convert a ROS2 Duration to seconds

        :param duration: The ROS2 Duration
        :type duration: Duration

        :return: The duration in seconds
        :rtype: float
        """
        return Duration.from_msg(duration).nanoseconds / 1e9

    def _init_articulation(self) -> None:
        """Initialize the articulation and register joints
        """
        # get articulation
        self._articulation = self.dci.get_articulation(self.articulation_path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: {} is not an articulation".format(path))
            return

        dof_props = self.dci.get_articulation_dof_properties(self._articulation)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        # get joints
        for i in range(self.dci.get_articulation_dof_count(self._articulation)):
            dof_ptr = self.dci.get_articulation_dof(self._articulation, i)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                dof_name = self.dci.get_dof_name(dof_ptr)
                if dof_name not in self._joints:
                    _joint = self.dci.find_articulation_joint(self._articulation, dof_name)
                    self._joints[dof_name] = {"joint": _joint,
                                              "type": self.dci.get_joint_type(_joint),
                                              "dof": self.dci.find_articulation_dof(self._articulation, dof_name),
                                              "lower": lower_limits[i],
                                              "upper": upper_limits[i],
                                              "has_limits": has_limits[i]}

        if not self._joints:
            print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: no joints found in {}".format(self.articulation_path))
            self.started = False

    def _set_joint_position(self, name: str, target_position: float) -> None:
        """Set the target position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param target_position: The target position
        :type target_position: float
        """
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        # scale target position for prismatic joints
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        # set target position
        self.dci.set_dof_position_target(self._joints[name]["dof"], target_position)

    def _get_joint_position(self, name: str) -> float:
        """Get the current position of a joint in the articulation

        :param name: The joint name
        :type name: str

        :return: The current position of the joint
        :rtype: float
        """
        position = self.dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS).pos
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            return position * get_stage_units()
        return position

    def _on_handle_accepted(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> None:
        """Callback function for handling newly accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle
        """
        goal_handle.execute()

    def _on_goal(self, goal: 'FollowJointTrajectory.Goal') -> 'rclpy.action.server.GoalResponse':
        """Callback function for handling new goal requests

        :param goal: The goal
        :type goal: FollowJointTrajectory.Goal

        :return: Whether the goal was accepted
        :rtype: rclpy.action.server.GoalResponse
        """
        # reject if joints don't match
        for name in goal.trajectory.joint_names:
            if name not in self._joints:
                print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: joints don't match ({} not in {})" \
                    .format(name, list(self._joints.keys())))
                return GoalResponse.REJECT

        # Preempt any in-flight goal so streaming clients can replace the
        # active trajectory. The previous _on_execute loop polls _action_goal
        # and exits with INVALID_GOAL once we clear it.
        if self._action_goal is not None:
            old_handle = self._action_goal_handle
            self._action_goal = None
            if old_handle is not None and old_handle.is_active:
                try:
                    old_handle.abort()
                except Exception as exc:
                    print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: abort failed: {}" \
                        .format(exc))
            self._action_goal_handle = None

        # Pad zero time_from_start so single-point trajectories become a
        # 2-point interpolation with enough physical time to converge.
        if goal.trajectory.points:
            first_tfs = goal.trajectory.points[0].time_from_start
            if first_tfs.sec == 0 and first_tfs.nanosec == 0:
                pad_total_ns = int(self._zero_time_pad_sec * 1e9)
                goal.trajectory.points[0].time_from_start.sec = pad_total_ns // 1_000_000_000
                goal.trajectory.points[0].time_from_start.nanosec = pad_total_ns % 1_000_000_000

        # check initial position
        if goal.trajectory.points[0].time_from_start:
            initial_point = JointTrajectoryPoint(
                positions=[
                    self._get_joint_position(name)
                    for name in goal.trajectory.joint_names
                ],
                time_from_start=Duration().to_msg(),
            )
            goal.trajectory.points.insert(0, initial_point)

        # reset internal data
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        self._action_point_index = 1

        # store goal data
        self._action_goal = goal

        return GoalResponse.ACCEPT

    def _on_trajectory_topic(self, msg: JointTrajectory) -> None:
        """Topic-interface entry point.

        Runs on the ROS executor thread. To avoid racing with the physics
        simulation on DCI calls, this method only validates the message and
        stashes it. step() picks it up on the main thread.
        """
        for name in msg.joint_names:
            if name not in self._joints:
                print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: topic msg joints don't match ({} not in {})" \
                    .format(name, list(self._joints.keys())))
                return
        if not msg.points:
            return
        with self._pending_lock:
            self._pending_topic_msg = msg

    def _install_pending_trajectory(self) -> None:
        """Promote a pending topic msg to the active trajectory (main thread only)."""
        with self._pending_lock:
            msg = self._pending_topic_msg
            self._pending_topic_msg = None
        if msg is None:
            return

        # Preempt any in-flight trajectory.
        if self._action_goal is not None:
            self._action_goal = None
            self._action_goal_handle = None

        # Single-point trajectories are setpoint commands ("go to this pose").
        # Bypass the interpolator entirely and write joint targets directly —
        # the DCI position controller handles physical convergence. Running a
        # state-machine interpolation per msg means each new streaming goal
        # preempts the previous one mid-flight (pumas head_node at 10 Hz with
        # 1 s pad → ~10% progress per cycle; pumas arm_node with 0.2 s →
        # ~50% per cycle), which makes the joint barely track the stream.
        # time_from_start on a single point is only a deadline hint, not a
        # forced interpolation duration.
        if len(msg.points) == 1:
            self.dci.wake_up_articulation(self._articulation)
            point = msg.points[0]
            for i, name in enumerate(msg.joint_names):
                if i < len(point.positions):
                    self._set_joint_position(name, point.positions[i])
            return

        # Multi-point / non-zero trajectory: run the interpolator.
        # Insert current state as t=0 anchor for interpolation.
        initial_point = JointTrajectoryPoint(
            positions=[
                self._get_joint_position(name) for name in msg.joint_names
            ],
            time_from_start=Duration().to_msg(),
        )
        msg.points.insert(0, initial_point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg
        self._action_start_time = self._node.get_clock().now().nanoseconds / 1e9
        self._action_result_message = None
        self._action_point_index = 1
        self._action_goal = goal
        # _action_goal_handle stays None -> step() treats this as topic mode

    def _on_cancel(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'rclpy.action.server.CancelResponse':
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: Whether the goal was canceled
        :rtype: rclpy.action.server.CancelResponse
        """
        if self._action_goal is None:
            return CancelResponse.REJECT
        # reset internal data
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        goal_handle.destroy()
        return CancelResponse.ACCEPT

    def _on_execute(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'FollowJointTrajectory.Result':
        """Callback function for processing accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: The result of the goal execution
        :rtype: FollowJointTrajectory.Result
        """
        # reset internal data
        self._action_start_time = self._node.get_clock().now().nanoseconds / 1e9
        self._action_result_message = None
        # set goal
        self._action_goal_handle = goal_handle
        # wait for the goal to be executed
        while self._action_result_message is None: 
            if self._action_goal is None:
                result = FollowJointTrajectory.Result()
                result.error_code = result.INVALID_GOAL
                return result
            time.sleep(self._action_dt)
        self._action_goal = None
        self._action_goal_handle = None
        return self._action_result_message

    def step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # Promote any pending topic-mode trajectory (received on the ROS
        # executor thread) so the DCI call for current joint state happens
        # here on the main thread.
        if self._pending_topic_msg is not None:
            self._install_pending_trajectory()
        # update articulation if a trajectory is active (action or topic mode)
        if self._action_goal is not None:
            self._action_dt = dt
            # end of trajectory
            if self._action_point_index >= len(self._action_goal.trajectory.points):
                handle = self._action_goal_handle
                self._action_goal = None
                self._action_result_message = FollowJointTrajectory.Result()
                self._action_result_message.error_code = self._action_result_message.SUCCESSFUL
                if handle is not None:
                    try:
                        handle.succeed()
                    except Exception:
                        pass
                    self._action_goal_handle = None
                return

            previous_point = self._action_goal.trajectory.points[self._action_point_index - 1]
            current_point = self._action_goal.trajectory.points[self._action_point_index]
            time_passed = self._node.get_clock().now().nanoseconds / 1e9 - self._action_start_time

            # set target using linear interpolation
            if time_passed <= self._duration_to_seconds(current_point.time_from_start):
                ratio = (time_passed - self._duration_to_seconds(previous_point.time_from_start)) \
                      / (self._duration_to_seconds(current_point.time_from_start) \
                          - self._duration_to_seconds(previous_point.time_from_start))
                self.dci.wake_up_articulation(self._articulation)
                for i, name in enumerate(self._action_goal.trajectory.joint_names):
                    side = -1 if current_point.positions[i] < previous_point.positions[i] else 1
                    target_position = previous_point.positions[i] \
                                    + side * ratio * abs(current_point.positions[i] - previous_point.positions[i])
                    self._set_joint_position(name, target_position)
            # send feedback
            else:
                self._action_point_index += 1
                # set joint targets for the new current point when advancing the index
                if self._action_point_index < len(self._action_goal.trajectory.points):
                    new_point = self._action_goal.trajectory.points[self._action_point_index]
                    self.dci.wake_up_articulation(self._articulation)
                    for i, name in enumerate(self._action_goal.trajectory.joint_names):
                        if i < len(new_point.positions):
                            target_position = new_point.positions[i]
                            self._set_joint_position(name, target_position)
                # feedback only when an action handle is attached
                if self._action_goal_handle is not None:
                    self._action_feedback_message.joint_names = list(self._action_goal.trajectory.joint_names)
                    self._action_feedback_message.actual.positions = [self._get_joint_position(name) \
                        for name in self._action_goal.trajectory.joint_names]
                    self._action_feedback_message.actual.time_from_start = Duration(seconds=time_passed).to_msg()
                    try:
                        self._action_goal_handle.publish_feedback(self._action_feedback_message)
                    except Exception:
                        pass
        # publish controller state for this timestep
        if self._state_pub is not None:
            msg = JointTrajectoryControllerState()
            msg.header.stamp = self._node.get_clock().now().to_msg()

            # Scope joint_names to this controller's joints when declared;
            # otherwise fall back to all articulation joints. Skip names that
            # aren't registered in self._joints (defensive).
            if self.controlled_joints:
                joint_names = [n for n in self.controlled_joints if n in self._joints]
            else:
                joint_names = sorted(self._joints.keys())
            msg.joint_names = joint_names

            # actual state from articulation
            actual_positions = []
            actual_velocities = []
            for name in joint_names:
                try:
                    pos = self._get_joint_position(name)
                except Exception:
                    pos = 0.0
                actual_positions.append(pos)
                try:
                    vel = self.dci.get_dof_state(
                        self._joints[name]["dof"],
                        _dynamic_control.STATE_VEL,
                    ).vel
                except Exception:
                    vel = 0.0
                actual_velocities.append(vel)

            msg.actual.positions = list(actual_positions)
            msg.actual.velocities = list(actual_velocities)

            # desired state from current trajectory point if a goal is active
            desired_positions = list(actual_positions)
            desired_velocities = [0.0] * len(joint_names)

            if self._action_goal is not None and self._action_goal.trajectory.points:
                # clamp index in case it's already at/after the end
                point_index = min(self._action_point_index, len(self._action_goal.trajectory.points) - 1)
                current_point = self._action_goal.trajectory.points[point_index]

                # map trajectory joint order to controller joint order
                for i, name in enumerate(self._action_goal.trajectory.joint_names):
                    if name in self._joints and name in joint_names:
                        j_idx = joint_names.index(name)
                        if i < len(current_point.positions):
                            desired_positions[j_idx] = current_point.positions[i]
                        if i < len(current_point.velocities):
                            desired_velocities[j_idx] = current_point.velocities[i]

            msg.desired.positions = desired_positions
            msg.desired.velocities = desired_velocities

            # error = desired - actual
            error_positions = [
                dp - ap for dp, ap in zip(desired_positions, actual_positions)
            ]
            error_velocities = [
                dv - av for dv, av in zip(desired_velocities, actual_velocities)
            ]
            msg.error.positions = error_positions
            msg.error.velocities = error_velocities

            # ROS 2 control_msgs/JointTrajectoryControllerState has BOTH the
            # legacy fields (actual/desired/error) and the new ones
            # (feedback/reference/error/output). pumas's arm_node reads
            # `feedback.positions[i]` without a size check and segfaults if
            # left empty — mirror the legacy data into the new fields.
            msg.feedback.positions = list(actual_positions)
            msg.feedback.velocities = list(actual_velocities)
            msg.reference.positions = list(desired_positions)
            msg.reference.velocities = list(desired_velocities)
            msg.output.positions = list(desired_positions)
            msg.output.velocities = list(desired_velocities)

            self._state_pub.publish(msg)
            if self._controller_state_pub is not None:
                self._controller_state_pub.publish(msg)

class RosControllerGripperCommand(RosController):
    def __init__(self,
                 node: Node,
                 _dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """GripperCommand interface

        :param node: The ROS node
        :type node: rclpy.node.Node
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(node)
        
        self.dci = _dci

        self._articulation = _dynamic_control.INVALID_HANDLE
        self._joints = {}

        self._action_server = None

        self._action_dt = 0.05
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        # TODO: add to schema?
        self._action_timeout = 10.0
        self._action_position_threshold = 0.001
        self._action_previous_position_sum = float("inf")

        # feedback / result
        self._action_result_message = None
        self._action_feedback_message = GripperCommand.Feedback()

    def start(self, _articulation_path, _action_topic_name) -> None:
        """Start the action server
        """
        print("[Info][semu.robotics.ros2_bridge] RosControllerGripperCommand: starting {}" \
            .format(_action_topic_name))

        # get attributes and relationships
        self.articulation_path = _articulation_path
        self.action_topic_name = _action_topic_name

        # start action server
        self._action_server = ActionServer(self._node,
                                           GripperCommand,
                                           self.action_topic_name,
                                           execute_callback=self._on_execute,
                                           goal_callback=self._on_goal,
                                           cancel_callback=self._on_cancel,
                                           handle_accepted_callback=self._on_handle_accepted)
        print("[Info][semu.robotics.ros2_bridge] RosControllerGripperCommand: register action {}" \
            .format(self.action_topic_name))

        self.started = True

    def stop(self) -> None:
        """Stop the action server
        """
        super().stop()
        self._articulation = _dynamic_control.INVALID_HANDLE
        # destroy action server
        if self._action_server is not None:
            print("[Info][semu.robotics.ros2_bridge] RosControllerGripperCommand: destroy action server")
            # self._action_server.destroy()
            self._action_server = None
        self._action_goal_handle = None
        self._action_goal = None

    def _duration_to_seconds(self, duration: Duration) -> float:
        """Convert a ROS2 Duration to seconds

        :param duration: The ROS2 Duration
        :type duration: Duration

        :return: The duration in seconds
        :rtype: float
        """
        return Duration.from_msg(duration).nanoseconds / 1e9

    def _init_articulation(self) -> None:
        """Initialize the articulation and register joints
        """
        # get articulation
        self._articulation = self.dci.get_articulation(self.articulation_path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print("[Warning][semu.robotics.ros2_bridge] RosControllerGripperCommand: {} is not an articulation".format(self.articulation_path))
            return

        dof_props = self.dci.get_articulation_dof_properties(self._articulation)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        # get joints
        for i in range(self.dci.get_articulation_dof_count(self._articulation)):
            dof_ptr = self.dci.get_articulation_dof(self._articulation, i)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                dof_name = self.dci.get_dof_name(dof_ptr)
                if dof_name not in self._joints:
                    _joint = self.dci.find_articulation_joint(self._articulation, dof_name)
                    self._joints[dof_name] = {"joint": _joint,
                                                "type": self.dci.get_joint_type(_joint),
                                                "dof": self.dci.find_articulation_dof(self._articulation, dof_name),
                                                "lower": lower_limits[i],
                                                "upper": upper_limits[i],
                                                "has_limits": has_limits[i]}

        if not self._joints:
            print("[Warning][semu.robotics.ros2_bridge] RosControllerGripperCommand: no joints found in {}".format(path))
            self.started = False

    def _set_joint_position(self, name: str, target_position: float) -> None:
        """Set the target position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param target_position: The target position
        :type target_position: float
        """
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        # scale target position for prismatic joints
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        # set target position
        self.dci.set_dof_position_target(self._joints[name]["dof"], target_position)

    def _get_joint_position(self, name: str) -> float:
        """Get the current position of a joint in the articulation

        :param name: The joint name
        :type name: str

        :return: The current position of the joint
        :rtype: float
        """
        position = self.dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS).pos
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            return position * get_stage_units()
        return position

    def _on_handle_accepted(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> None:
        """Callback function for handling newly accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle
        """
        goal_handle.execute()

    def _on_goal(self, goal: 'GripperCommand.Goal') -> 'rclpy.action.server.GoalResponse':
        """Callback function for handling new goal requests

        :param goal: The goal
        :type goal: GripperCommand.Goal

        :return: Whether the goal was accepted
        :rtype: rclpy.action.server.GoalResponse
        """
        # reject if there is an active goal
        if self._action_goal is not None:
            print("[Warning][semu.robotics.ros2_bridge] RosControllerGripperCommand: multiple goals not supported")
            return GoalResponse.REJECT

        # reset internal data
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        self._action_previous_position_sum = float("inf")

        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'rclpy.action.server.CancelResponse':
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: Whether the goal was canceled
        :rtype: rclpy.action.server.CancelResponse
        """
        if self._action_goal is None:
            return CancelResponse.REJECT
        # reset internal data
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        self._action_previous_position_sum = float("inf")
        goal_handle.destroy()
        return CancelResponse.ACCEPT

    def _on_execute(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'GripperCommand.Result':
        """Callback function for processing accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: The result of the goal execution
        :rtype: GripperCommand.Result
        """
        # reset internal data
        self._action_start_time = self._node.get_clock().now().nanoseconds / 1e9
        self._action_result_message = None
        self._action_previous_position_sum = float("inf")
        # set goal
        self._action_goal_handle = goal_handle
        self._action_goal = goal_handle.request
        # wait for the goal to be executed
        while self._action_result_message is None: 
            if self._action_goal is None:
                return GripperCommand.Result()
            time.sleep(self._action_dt)
        self._action_goal = None
        self._action_goal_handle = None
        return self._action_result_message

    def step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # update articulation
        if self._action_goal is not None and self._action_goal_handle is not None:
            self._action_dt = dt
            target_position = self._action_goal.command.position
            # set target
            self.dci.wake_up_articulation(self._articulation)
            for name in self._joints:
                self._set_joint_position(name, target_position)
            # end (position reached)
            position = 0
            current_position_sum = 0
            position_reached = True
            for name in self._joints:
                position = self._get_joint_position(name)
                current_position_sum += position
                if abs(position - target_position) > self._action_position_threshold:
                    position_reached = False
                    break
            if position_reached:
                self._action_result_message = GripperCommand.Result()
                self._action_result_message.position = position
                self._action_result_message.stalled = False
                self._action_result_message.reached_goal = True
                if self._action_goal_handle is not None:
                    self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                return
            # end (stalled)
            if abs(current_position_sum - self._action_previous_position_sum) < 1e-6:
                self._action_result_message = GripperCommand.Result()
                self._action_result_message.position = position
                self._action_result_message.stalled = True
                self._action_result_message.reached_goal = False
                if self._action_goal_handle is not None:
                    self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                return
            self._action_previous_position_sum = current_position_sum
            # end (timeout)
            time_passed = self._node.get_clock().now().nanoseconds / 1e9 - self._action_start_time
            if time_passed >= self._action_timeout:
                self._action_result_message = GripperCommand.Result()
                if self._action_goal_handle is not None:
                    self._action_goal_handle.abort()
                    self._action_goal_handle = None
            # TODO: send feedback
            # self._action_goal_handle.publish_feedback(self._action_feedback_message)
