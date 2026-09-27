"""
Voice commander node: /voice_command (wav path) -> ASR -> LLM -> normaliser -> MoveRobot goal.

Concurrency design
  - AI work (Whisper + Phi-3.5, optionally +N s injected latency) runs in a
    ThreadPoolExecutor, NEVER on the ROS executor.
  - Finished AI results go into a thread-safe queue; a 50 Hz timer on the
    executor drains it and dispatches goals. So the executor only ever does
    short, non-blocking work, and telemetry, feedback and cancels keep flowing.
  - MultiThreadedExecutor + separate callback groups for subscriptions,
    the dispatch timer and the action client.

Arbitration policy
  - Every voice command gets a sequence number; latest command wins.
    An AI result older than the last dispatched command is dropped.
  - "Stop" fast path: if the transcript is a stop word, skip the LLM and
    cancel immediately.
  - A new move/turn cancels (preempts) the active goal before sending.
  - Rejected/unsupported commands leave the current goal untouched.

Stale-callback handling
  - Exactly one _active_goal_id is authoritative. The moment a goal is
    cancelled or superseded, _active_goal_id is cleared, so ANY later
    feedback/result from it is logged as [stale] and ignored.
  - A goal accepted after it was already superseded is cancelled on arrival.
"""
import queue
import re
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from std_msgs.msg import String, Empty
from action_msgs.msg import GoalStatus

from commander_interfaces.action import MoveRobot
from voice_commander.ai_pipeline import AIPipeline
from voice_commander.normaliser import normalise, NormalisationError, RobotCommand

STOP_WORDS = {"stop", "halt", "freeze"}
STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def gid(uuid) -> str:
    return bytes(uuid).hex()[:8]


class VoiceCommander(Node):
    def __init__(self):
        super().__init__("voice_commander")
        self.declare_parameter("ai_latency_s", 0.0)
        self.declare_parameter("whisper_model", "tiny.en")
        self.declare_parameter("llm_model", "phi3.5")
        self._latency = float(self.get_parameter("ai_latency_s").value)

        self._pipe = AIPipeline(
            whisper_model=self.get_parameter("whisper_model").value,
            llm_model=self.get_parameter("llm_model").value)
        self.get_logger().info("Warming up LLM...")
        self._pipe.warmup()

        self._workers = ThreadPoolExecutor(max_workers=2, thread_name_prefix="ai")
        self._results = queue.Queue()

        self._lock = threading.Lock()
        self._seq = 0                    # last issued voice command number
        self._last_dispatched_seq = 0    # only touched by the dispatch timer
        self._active_goal_id = None      # bytes: the ONLY goal allowed to change state
        self._active_goal_handle = None
        self._send_token = 0
        self._pending_token = None       # goal sent but not yet accepted
        self._state_count = 0

        self._client = ActionClient(self, MoveRobot, "move_robot",
                                    callback_group=ReentrantCallbackGroup())
        self.create_subscription(String, "voice_command", self._on_voice, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(Empty, "cancel_goal", self._on_manual_cancel, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(String, "robot_state", self._on_state, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.02, self._drain_results,
                          callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(f"Voice commander ready (ai_latency_s={self._latency})")

    # ---------------- inputs ----------------
    def _on_voice(self, msg: String):
        with self._lock:
            self._seq += 1
            seq = self._seq
        self.get_logger().info(f"[voice #{seq}] received {msg.data} -> AI worker (executor NOT blocked)")
        self._workers.submit(self._ai_job, seq, msg.data)

    def _on_manual_cancel(self, _msg):
        self.get_logger().warn("[operator] manual cancel (no AI involved)")
        self._cancel_active("operator cancel")

    def _on_state(self, msg: String):
        self._state_count += 1
        if self._state_count % 10 == 0:
            self.get_logger().info(f"[telemetry@commander] {msg.data}")

    # ---------------- AI worker thread ----------------
    def _ai_job(self, seq, path):
        t0 = time.monotonic()
        try:
            transcript = self._pipe.transcribe(path)
            self.get_logger().info(f"[voice #{seq}] transcript='{transcript}'")
            words = re.findall(r"[a-z]+", transcript.lower())
            if words and words[0] in STOP_WORDS and len(words) <= 3:
                self._results.put((seq, RobotCommand("stop"), "STOP fast-path (LLM skipped)",
                                   time.monotonic() - t0))
                return
            raw = self._pipe.interpret(transcript)
            if self._latency > 0:
                self.get_logger().info(f"[voice #{seq}] injecting {self._latency:.1f}s AI latency")
                time.sleep(self._latency)
            try:
                cmd, note = normalise(raw), "LLM + normaliser"
            except NormalisationError as e:
                cmd, note = None, f"REJECTED by normaliser: {e} (llm_raw={raw.strip()})"
        except Exception as e:   # AI failure must never crash the robot side
            cmd, note = None, f"AI error: {e}"
        self._results.put((seq, cmd, note, time.monotonic() - t0))

    # ---------------- dispatch (executor timer, short & non-blocking) ----------------
    def _drain_results(self):
        while True:
            try:
                seq, cmd, note, dt = self._results.get_nowait()
            except queue.Empty:
                return
            if seq < self._last_dispatched_seq:
                self.get_logger().warn(
                    f"[arbitration] dropping stale AI result #{seq} "
                    f"(newer command #{self._last_dispatched_seq} already handled)")
                continue
            self._last_dispatched_seq = seq
            if cmd is None:
                self.get_logger().warn(f"[voice #{seq}] {note} -> robot state unchanged")
                continue
            self.get_logger().info(f"[voice #{seq}] {note} in {dt:.2f}s -> {cmd}")
            self._dispatch(cmd, seq)

    def _dispatch(self, cmd: RobotCommand, seq: int):
        if cmd.kind == "stop":
            self._cancel_active(f"voice stop #{seq}")
            return
        if not self._client.server_is_ready():
            self.get_logger().error("[goal] action server not available")
            return
        self._cancel_active(f"preempted by voice #{seq}")
        with self._lock:
            self._send_token += 1
            token = self._send_token
            self._pending_token = token
        goal = MoveRobot.Goal(command_type=cmd.kind, distance_m=cmd.distance_m, yaw_rad=cmd.yaw_rad)
        fut = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(partial(self._on_goal_response, token, seq))

    def _cancel_active(self, reason: str):
        with self._lock:
            gh = self._active_goal_handle
            self._active_goal_handle = None
            self._active_goal_id = None     # from now on its callbacks are stale
            self._pending_token = None      # a not-yet-accepted goal is also superseded
        if gh is not None:
            self.get_logger().warn(f"[arbitration] cancelling goal {gid(gh.goal_id.uuid)} ({reason})")
            gh.cancel_goal_async()
        else:
            self.get_logger().info(f"[arbitration] {reason}: no active goal")

    # ---------------- action client callbacks ----------------
    def _on_goal_response(self, token, seq, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn(f"[goal] voice #{seq} REJECTED by robot (safety interlock?)")
            return
        uid = bytes(gh.goal_id.uuid)
        with self._lock:
            stale = token != self._pending_token
            if not stale:
                self._pending_token = None
                self._active_goal_handle = gh
                self._active_goal_id = uid
        if stale:
            self.get_logger().warn(f"[stale] goal {gid(uid)} (voice #{seq}) accepted after being superseded -> cancelling it")
            gh.cancel_goal_async()
        else:
            self.get_logger().info(f"[goal {gid(uid)}] ACTIVE (voice #{seq})")
        gh.get_result_async().add_done_callback(partial(self._on_result, uid))

    def _on_feedback(self, fb_msg):
        uid = bytes(fb_msg.goal_id.uuid)
        with self._lock:
            active = self._active_goal_id
        if uid != active:
            self.get_logger().warn(
                f"[stale] dropped feedback from goal {gid(uid)} "
                f"(active={gid(active) if active else 'none'})")
            return
        f = fb_msg.feedback
        self.get_logger().info(
            f"[feedback {gid(uid)}] x={f.x:.2f} y={f.y:.2f} yaw={f.yaw:.3f} progress={f.progress * 100:.0f}%",
            throttle_duration_sec=1.0)

    def _on_result(self, uid, future):
        res = future.result()
        status = STATUS_NAMES.get(res.status, str(res.status))
        r = res.result
        with self._lock:
            active = self._active_goal_id
            is_active = uid == active
            if is_active:
                self._active_goal_id = None
                self._active_goal_handle = None
        if not is_active:
            self.get_logger().warn(
                f"[stale] ignored late result from goal {gid(uid)} ({status}/{r.status}) "
                f"-> active goal {gid(active) if active else 'none'} unaffected")
            return
        self.get_logger().info(
            f"[goal {gid(uid)}] FINISHED {status} robot_status={r.status} "
            f"distance={r.distance_traveled_m:.2f}m yaw={r.yaw_turned_rad:.3f}rad")

    def destroy_node(self):
        self._workers.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main():
    rclpy.init()
    node = VoiceCommander()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()