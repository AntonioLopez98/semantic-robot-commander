#!/usr/bin/env python3
"""
Scenario driver: plays the 'user' and the 'hardware'.
Publishes voice commands (/voice_command), hardware obstacle events
(/obstacle_detected) and operator cancels (/cancel_goal) with precise timing,
reacting to the robot's telemetry (/robot_state).
"""
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, Empty, String

AUDIO = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "src", "voice_commander", "audio"))


class Driver(Node):
    def __init__(self):
        super().__init__("scenario_driver")
        self._voice = self.create_publisher(String, "voice_command", 10)
        self._obstacle = self.create_publisher(Bool, "obstacle_detected", 10)
        self._cancel = self.create_publisher(Empty, "cancel_goal", 10)
        self.active = None   # None until the first telemetry message arrives
        self.create_subscription(String, "robot_state", self._on_state, 10)

    def _on_state(self, msg):
        for tok in msg.data.split():
            if tok.startswith("active_goal="):
                self.active = tok.split("=", 1)[1]

    # ---------- actions ----------
    def say(self, wav):
        self.get_logger().info(f">>> USER says: {wav}")
        self._voice.publish(String(data=os.path.join(AUDIO, wav)))

    def obstacle(self, value):
        self.get_logger().warn(f">>> HARDWARE: obstacle_detected={value}")
        self._obstacle.publish(Bool(data=value))

    def cancel(self):
        self.get_logger().warn(">>> OPERATOR: /cancel_goal (no AI involved)")
        self._cancel.publish(Empty())

    def note(self, text):
        self.get_logger().info(f"--- {text}")

    # ---------- waits ----------
    def wait_ready(self, timeout=30.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if (self.active is not None
                    and self._voice.get_subscription_count() > 0
                    and self._obstacle.get_subscription_count() > 0):
                time.sleep(0.5)
                return
            time.sleep(0.1)
        raise TimeoutError("robot/commander not ready")

    def wait_active(self, not_this=None, timeout=40.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.active not in (None, "none", not_this):
                self.note(f"goal {self.active} is executing")
                return self.active
            time.sleep(0.02)
        raise TimeoutError("no goal became active")

    def wait_idle(self, timeout=40.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.active == "none":
                self.note("robot idle")
                return
            time.sleep(0.05)
        raise TimeoutError("robot did not become idle")


# ======================= scenarios =======================

def scenario_1(d):
    """Asynchronous hardware interrupt."""
    d.say("my_move_forward.wav")
    d.wait_active()
    d.note("waiting 2.0 s into execution...")
    time.sleep(2.0)
    d.obstacle(True)
    d.note("robot must stay stopped with NO new user command / AI involvement")
    time.sleep(3.0)
    d.obstacle(False)


def scenario_2(d):
    """Coordinate and unit normalisation."""
    d.say("my_turn_left.wav")
    d.wait_active()
    d.wait_idle()


def scenario_3(d):
    """High-latency AI processing (commander runs with ai_latency_s=3.0)."""
    d.say("my_move_forward.wav")
    d.wait_active()
    time.sleep(1.0)
    d.say("my_turn_right.wav")
    d.note("AI now busy ~6 s (ASR + LLM + 3 s injected); feedback/telemetry must continue")
    time.sleep(1.5)
    d.cancel()
    d.note("cancel issued WHILE AI is busy -> move must stop now")
    time.sleep(1.0)
    d.obstacle(True)
    d.note("safety event WHILE AI is busy -> must be handled now")
    d.note("when the turn result arrives, the robot must refuse it (obstacle interlock)")
    time.sleep(6.0)
    d.obstacle(False)


def scenario_4(d):
    """Voice preemption: 'Stop.' during a long move."""
    d.say("my_move_forward.wav")
    d.wait_active()
    time.sleep(3.0)
    d.say("my_stop.wav")
    time.sleep(3.0)


def scenario_5(d):
    """Cancellation race (robot runs with late_result_delay_s=1.5)."""
    d.say("my_move_forward.wav")
    goal_a = d.wait_active()
    d.note(f"Goal A = {goal_a}")
    time.sleep(3.0)
    d.say("my_turn_left.wav")
    d.note("Goal B will supersede A; A's feedback/result are delivered 1.5 s LATE")
    goal_b = d.wait_active(not_this=goal_a)
    d.note(f"Goal B = {goal_b}; late callbacks from A must NOT change B's state")
    d.wait_idle()


SCENARIOS = {1: scenario_1, 2: scenario_2, 3: scenario_3, 4: scenario_4, 5: scenario_5}


def main():
    n = int(sys.argv[1])
    rclpy.init()
    d = Driver()
    ex = SingleThreadedExecutor()
    ex.add_node(d)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()
    try:
        d.wait_ready()
        d.note(f"===== SCENARIO {n}: {SCENARIOS[n].__doc__} =====")
        SCENARIOS[n](d)
        time.sleep(1.5)
        d.note(f"===== SCENARIO {n} DONE =====")
    finally:
        ex.shutdown()                 # stop spinning first
        spin_thread.join(timeout=2.0)
        d.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()