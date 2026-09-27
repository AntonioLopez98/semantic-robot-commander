"""
Mock robot: MoveRobot action server + telemetry + independent safety watchdog.

Concurrency design:
  - MultiThreadedExecutor with three callback groups:
      * action group (Reentrant): goal/cancel/execute callbacks
      * safety group (MutuallyExclusive): /obstacle_detected subscription
      * telemetry group (MutuallyExclusive): 10 Hz state publisher
    so a running goal never blocks safety or telemetry.
  - All pose updates happen under one lock, and the safety callback flips
    the obstacle flag under the same lock -> no motion step can happen
    after the callback returns ("immediate" stop).

Arbitration: single active goal. A newly accepted goal becomes the active
one; the old goal's execute loop notices on its next tick that it is no
longer active and ends as "preempted" without moving the robot again.
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from std_msgs.msg import Bool, String

from commander_interfaces.action import MoveRobot

LINEAR_SPEED = 0.5    # m/s
ANGULAR_SPEED = 0.5   # rad/s
SIM_DT = 0.05         # 20 Hz simulation tick
FEEDBACK_EVERY = 4    # feedback at 5 Hz


def short_id(goal_handle):
    return bytes(goal_handle.goal_id.uuid).hex()[:8]


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        self._lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._active_handle = None   # only this goal may move the robot
        self._obstacle = False

        # Fault injection (Scenario 5): delay the final feedback/result of a
        # cancelled/preempted goal so it arrives while the NEXT goal is running.
        self.declare_parameter('late_result_delay_s', 0.0)
        self._late_delay = float(self.get_parameter('late_result_delay_s').value)

        action_group = ReentrantCallbackGroup()
        safety_group = MutuallyExclusiveCallbackGroup()
        telemetry_group = MutuallyExclusiveCallbackGroup()

        self._server = ActionServer(
            self, MoveRobot, 'move_robot',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            handle_accepted_callback=self.accepted_cb,
            cancel_callback=self.cancel_cb,
            callback_group=action_group)

        self.create_subscription(Bool, 'obstacle_detected', self.obstacle_cb, 10,
                                 callback_group=safety_group)
        self._state_pub = self.create_publisher(String, 'robot_state', 10)
        self._tel_count = 0
        self.create_timer(0.1, self.telemetry_cb, callback_group=telemetry_group)

        self.get_logger().info('Mock robot ready (action: /move_robot, safety: /obstacle_detected)')

    # ---------------- safety watchdog ----------------
    def obstacle_cb(self, msg: Bool):
        t_detect = time.monotonic()
        with self._lock:
            was = self._obstacle
            self._obstacle = msg.data
            active = self._active_handle
            pose = (self.x, self.y, self.yaw)
        if msg.data and not was:
            gid = short_id(active) if active else 'none'
            self.get_logger().warn(
                f'[SAFETY] obstacle_detected=True -> motion HALTED at '
                f'x={pose[0]:.2f} y={pose[1]:.2f} yaw={pose[2]:.2f} (active goal {gid}) '
                f't={t_detect:.3f}')
        elif not msg.data and was:
            self.get_logger().info('[SAFETY] obstacle cleared, new goals allowed')

    # ---------------- telemetry ----------------
    def telemetry_cb(self):
        with self._lock:
            active = short_id(self._active_handle) if self._active_handle else 'none'
            s = (f'x={self.x:.2f} y={self.y:.2f} yaw={self.yaw:.2f} '
                 f'active_goal={active} obstacle={self._obstacle}')
        self._state_pub.publish(String(data=s))
        self._tel_count += 1
        if self._tel_count % 10 == 0:   # log once per second
            self.get_logger().info(f'[telemetry] {s}')

    # ---------------- action callbacks ----------------
    def goal_cb(self, goal_request):
        with self._lock:
            if self._obstacle:
                self.get_logger().warn('[goal] REJECTED: obstacle present')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def accepted_cb(self, goal_handle):
        with self._lock:
            old = self._active_handle
            self._active_handle = goal_handle
        if old is not None and old.is_active:
            self.get_logger().info(
                f'[arbitration] goal {short_id(goal_handle)} preempts {short_id(old)}')
        goal_handle.execute()

    def cancel_cb(self, goal_handle):
        self.get_logger().info(f'[cancel] cancel requested for goal {short_id(goal_handle)}')
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        g = goal_handle.request
        gid = short_id(goal_handle)
        result = MoveRobot.Result()

        if g.command_type == 'move':
            target, speed, sign = abs(g.distance_m), LINEAR_SPEED, math.copysign(1.0, g.distance_m)
        elif g.command_type == 'turn':
            target, speed, sign = abs(g.yaw_rad), ANGULAR_SPEED, math.copysign(1.0, g.yaw_rad)
        else:
            self.get_logger().error(f'[goal {gid}] unknown command_type "{g.command_type}"')
            goal_handle.abort()
            result.status = 'aborted'
            return result

        self.get_logger().info(
            f'[goal {gid}] START {g.command_type} distance_m={g.distance_m:.2f} '
            f'yaw_rad={g.yaw_rad:.3f}')

        done = 0.0
        tick = 0
        status = 'succeeded'
        fb = MoveRobot.Feedback()
        while True:
            with self._lock:
                if self._active_handle is not goal_handle:
                    status = 'preempted'
                    break
                if goal_handle.is_cancel_requested:
                    status = 'canceled'
                    break
                if self._obstacle:
                    status = 'safety_stop'
                    break
                if done >= target - 1e-9:
                    status = 'succeeded'
                    break
                step = min(speed * SIM_DT, target - done)
                if g.command_type == 'move':
                    self.x += sign * step * math.cos(self.yaw)
                    self.y += sign * step * math.sin(self.yaw)
                else:
                    self.yaw = wrap_angle(self.yaw + sign * step)
                done += step
                fb = MoveRobot.Feedback(x=self.x, y=self.y, yaw=self.yaw,
                                        progress=done / target if target > 0 else 1.0)
            tick += 1
            if tick % FEEDBACK_EVERY == 0:
                goal_handle.publish_feedback(fb)
            time.sleep(SIM_DT)
        if status in ('canceled', 'preempted') and self._late_delay > 0:
            self.get_logger().warn(
                f'[fault-injection] goal {gid} ended as {status}; delivering its last '
                f'feedback + result {self._late_delay:.1f}s LATE')
            time.sleep(self._late_delay)
            goal_handle.publish_feedback(fb)   # "zombie" feedback from a dead goal
        # terminal state
        if status == 'succeeded':
            goal_handle.succeed()
        elif status == 'canceled':
            goal_handle.canceled()
        else:   # preempted or safety_stop
            goal_handle.abort()

        with self._lock:
            if self._active_handle is goal_handle:
                self._active_handle = None

        if g.command_type == 'move':
            result.distance_traveled_m = sign * done
        else:
            result.yaw_turned_rad = sign * done
        result.status = status
        self.get_logger().info(f'[goal {gid}] END status={status} done={done:.2f}')
        return result


def main():
    rclpy.init()
    node = MockRobot()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()