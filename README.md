# Safety-Aware Semantic Robot Commander (ROS 2 Jazzy)

Natural-language voice commands → structured intent → validated robot actions, with the AI kept strictly outside the real-time control path.

```
Audio ──► Whisper Tiny ──► Phi-3.5 Mini ──► Deterministic ──► ROS 2 Action ──► Mock Action Server
(.wav)     (ASR)            (LLM, JSON       Normaliser        Client            (robot + telemetry
                             intent)         (SI, REP-103,     (arbitration,      + safety watchdog)
                                              bounds)           stale filtering)
```

**Guiding principle: the LLM interprets, the deterministic layer decides, the robot protects itself.**

---

## Results (all 5 required scenarios pass)

Full logs are in `logs/scenarioN/` (`robot.log`, `commander.log`, `driver.log`, and `timeline.log`, which merges all three sorted by timestamp).

| # | Scenario | Evidence from logs |
|---|----------|--------------------|
| 1 | Asynchronous hardware interrupt | `obstacle_detected=True` published exactly 2.0 s into a 10 m move; robot halted **5–14 ms** later (`status=safety_stop`), with no AI or user involvement. |
| 2 | Coordinate & unit normalisation | Spoken "Turn 90 degrees to the left" → LLM `{"angle": 90, "angle_unit": "degrees", "direction": "left"}` → normaliser `yaw_rad = +1.5708` (REP-103: left = CCW = +yaw) → robot turns to 1.571 rad. |
| 3 | High-latency AI (3 s injected) | While the AI was busy for ~5.4 s: action feedback and telemetry kept flowing, an operator cancel stopped the move in ~0.12 s, an obstacle was handled in 3 ms, and the late AI result was **rejected by the robot's safety interlock**. |
| 4 | Voice preemption ("Stop.") | "Stop." during a 10 m move → STOP fast-path (LLM skipped) → goal cancelled; **~0.37 s** from voice command to halted robot. |
| 5 | Cancellation race | Goal A superseded by Goal B; A's last feedback and result were deliberately delivered 1.5 s late (fault injection) while B was running → both logged as `[stale]` and dropped; B finished `SUCCEEDED` unaffected. |

Additional evidence: `logs/normaliser_selftest.log` (12 normalisation/rejection cases) and `logs/ai_pipeline_test.log` (real voice recordings through ASR → LLM → normaliser).

---

## Repository layout

```
setup.sh                    one-command install + build
requirements.txt            Python dependencies (pip)
src/
  commander_interfaces/     MoveRobot.action (goal in SI units + REP-103 only)
  mock_robot/               mock_robot_node.py: action server, 10 Hz telemetry, safety watchdog
  voice_commander/
    voice_commander/
      ai_pipeline.py        Whisper Tiny (faster-whisper) + Phi-3.5 via Ollama (ROS-free)
      normaliser.py         deterministic validation/conversion (ROS-free, no AI)
      voice_commander_node.py   ROS node: async AI, arbitration, stale-callback filtering
    audio/                  pre-recorded voice commands (my_*.wav)
scenarios/
  driver.py                 plays "user" + "hardware" with precise, telemetry-triggered timing
  run_scenario.sh           starts both nodes, runs a scenario, saves logs
logs/                       evidence for every scenario
```

---

## How to run

Tested on Windows 11 + WSL2, Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, NVIDIA RTX A2000 Laptop GPU (8 GB). An NVIDIA GPU is recommended for the LLM but not required (Ollama falls back to CPU, with slower responses).

### 1. Quick setup

Requires Ubuntu 24.04 with ROS 2 Jazzy installed in `/opt/ros/jazzy`. The repository root is the ROS 2 workspace, so it can be cloned anywhere.

```bash
git clone https://github.com/AntonioLopez98/semantic-robot-commander.git ros2_ws
cd ros2_ws
./setup.sh
source install/setup.bash
```

`setup.sh` installs the system packages, the Python dependencies from `requirements.txt` (faster-whisper, pydantic, requests), Ollama and the Phi-3.5 Mini Instruct model (Ollama's default 4-bit build, ~2.2 GB), then builds the workspace. Whisper Tiny (`tiny.en`) is downloaded automatically from Hugging Face on first use.

<details>
<summary>Manual setup (what setup.sh does)</summary>

```bash
sudo apt install -y python3-pip python3-colcon-common-extensions curl zstd
pip install --user --break-system-packages -r requirements.txt
curl -fsSL https://ollama.com/install.sh | sh
ollama pull phi3.5
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If `ollama pull` cannot connect to the server (e.g. WSL without systemd), run `ollama serve` in another terminal first.
</details>

### 2. Run the required scenarios (automated, logs saved)

```bash
cd scenarios
./run_scenario.sh 1   # hardware interrupt
./run_scenario.sh 2   # unit/coordinate normalisation
./run_scenario.sh 3   # high-latency AI (commander runs with ai_latency_s=3.0)
./run_scenario.sh 4   # voice "Stop." preemption
./run_scenario.sh 5   # cancellation race (robot runs with late_result_delay_s=1.5)
```

Each run prints the key events and saves the logs in `logs/scenarioN/`.

### 3. Manual mode (optional)

Run each command from the repository root, in a terminal where `install/setup.bash` has been sourced.

```bash
# Terminal 1
ros2 run mock_robot mock_robot_node
# Terminal 2
ros2 run voice_commander voice_commander_node --ros-args -p ai_latency_s:=0.0
# Terminal 3
A=$(pwd)/src/voice_commander/audio
ros2 topic pub --once /voice_command std_msgs/msg/String "{data: '$A/my_move_forward.wav'}"
ros2 topic pub --once /voice_command std_msgs/msg/String "{data: '$A/my_stop.wav'}"
ros2 topic pub --once /obstacle_detected std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /cancel_goal std_msgs/msg/Empty "{}"
```

Standalone tests of the non-ROS layers:

```bash
python3 src/voice_commander/voice_commander/normaliser.py
cd src/voice_commander/voice_commander && python3 ai_pipeline.py ../audio
```

### Interfaces

| Name | Type | Direction | Purpose |
|------|------|-----------|---------|
| `/move_robot` | `commander_interfaces/action/MoveRobot` | commander → robot | move/turn goal (SI units, REP-103), feedback (pose, progress), result (status) |
| `/voice_command` | `std_msgs/String` | user → commander | path of an audio file to process |
| `/obstacle_detected` | `std_msgs/Bool` | hardware → robot | independent safety input |
| `/cancel_goal` | `std_msgs/Empty` | operator → commander | non-AI cancel |
| `/robot_state` | `std_msgs/String` | robot → all | 10 Hz telemetry |

| Parameter | Node | Default | Purpose |
|-----------|------|---------|---------|
| `ai_latency_s` | voice_commander | 0.0 | injected AI delay (Scenario 3) |
| `whisper_model` / `llm_model` | voice_commander | `tiny.en` / `phi3.5` | model selection |
| `late_result_delay_s` | mock_robot | 0.0 | fault injection: delay final feedback/result of cancelled/preempted goals (Scenario 5) |

---

## Architecture overview

The system has two ROS nodes in **separate processes**, plus two ROS-free Python modules:

- **`ai_pipeline.py` (non-deterministic, slow).** Whisper Tiny (`tiny.en` via faster-whisper, CPU int8) transcribes the audio. Phi-3.5 Mini (via Ollama, GPU, `format: json`, `temperature: 0`) turns the text into a **semantic intent** in human terms, e.g. `{"action": "turn", "angle": 90, "angle_unit": "degrees", "direction": "left"}`. The LLM is explicitly told *not* to convert units or choose sign conventions.
- **`normaliser.py` (deterministic, fast, pure Python).** It is the only path from an intent to a robot command. It extracts JSON from noisy LLM text (code fences, chatter), validates it against a pydantic schema (case-insensitive, string numbers coerced), converts units (m/cm/mm/ft/in; degrees/radians), applies REP-103 (+x forward, +yaw = counter-clockwise = left), and enforces bounds (|distance| ≤ 20 m, |yaw| ≤ 2π). **Policy: anything malformed, ambiguous, unsupported or out of bounds is rejected, never guessed or silently clamped.** A rejected command leaves the robot's current goal untouched.
- **`voice_commander_node`** orchestrates everything asynchronously, owns goal arbitration and filters stale callbacks.
- **`mock_robot_node`** simulates non-instantaneous motion (0.5 m/s, 0.5 rad/s, 20 Hz integration), publishes 10 Hz telemetry and 5 Hz action feedback, supports cancellation and preemption, and contains the safety watchdog.

The `MoveRobot` action goal only carries already-normalised SI/REP-103 values, so the robot never needs to interpret language or ambiguous units.

---

## Concurrency / executor design

**Commander node**

- AI work (ASR → LLM → optional injected latency → normalisation) runs in a `ThreadPoolExecutor` (2 workers), **never on the ROS executor**.
- Finished results are placed on a thread-safe `queue.Queue`. A 50 Hz timer on the executor drains it and dispatches goals. Executor callbacks therefore only do short, non-blocking work.
- `MultiThreadedExecutor` (6 threads) with separate callback groups: a Reentrant group for the action client (goal response, feedback, result), and MutuallyExclusive groups for `/voice_command`, `/cancel_goal`, `/robot_state` and the dispatch timer. A slow callback in one group can't starve another.
- Shared goal-tracking state is protected by a single lock, held only for a few assignments.
- The heavy AI computation runs in native code (CTranslate2) or out of process (Ollama over HTTP), both of which release Python's GIL, so the executor threads stay responsive while the AI works (see the Scenario 3 logs).

**Robot node**

- `MultiThreadedExecutor` (8 threads) with three callback groups: action (Reentrant), safety subscription (MutuallyExclusive) and telemetry timer (MutuallyExclusive). A long-running goal loop never blocks safety or telemetry.
- All pose updates happen under one lock. The loop sleeps *outside* the lock, so the safety callback can always acquire it within one tick.

---

## Goal arbitration policy

1. **Single active goal.** The commander tracks exactly one `_active_goal_id`. The robot tracks exactly one `_active_handle`, and only that handle may move the robot.
2. **Latest command wins.** Every voice command gets a sequence number. If an AI result arrives after a *newer* command has already been handled, it is dropped (`[arbitration] dropping stale AI result`). This also covers a slow "move" finishing after a fast "stop".
3. **Preemption.** A new valid move/turn first cancels the active goal, then sends the new one. On the server side, a newly accepted goal becomes the active handle, and the previous execute loop notices on its next tick and ends as `preempted` without moving the robot again. Either path alone is sufficient; Scenario 5 shows the server-side path.
4. **STOP fast-path.** If the transcript is a stop word ("stop", "halt", "freeze"; ≤ 3 words), the LLM is skipped and the active goal is cancelled immediately (~0.3 s after ASR instead of ~2.5 s).
5. **Operator cancel** (`/cancel_goal`) bypasses AI entirely.
6. **Rejected or unsupported commands** do not affect the current goal.
7. **Safety interlock.** While `obstacle_detected` is true, the robot rejects new goals, so a late AI result can never restart motion (Scenario 3).

---

## Safety / watchdog design

- The watchdog lives **inside the robot node**, in a different process from the AI and the commander. It depends on nothing but the `/obstacle_detected` topic. It keeps working if the commander is slow, blocked or crashed.
- The safety callback runs in its own MutuallyExclusive callback group on a multi-threaded executor, so a long-running goal can never delay it.
- The obstacle flag is set under the same lock that guards every motion step. **Once the callback has run, no further motion step can occur.** The execute loop then ends the goal as `safety_stop`, which the client receives as `ABORTED`. In the logs, `END status=safety_stop` sometimes prints *before* the `[SAFETY]` warning, because the motion loop reacts before the callback reaches its log line.
- Measured reaction time: 3–14 ms from the obstacle message being published to motion stopping.
- **Latch and interlock:** new goals are rejected while the obstacle is present. Clearing the obstacle does not resume the old goal; a new command is required.

---

## Stale callback handling

- The commander treats exactly one goal ID as authoritative. **The instant a goal is cancelled or superseded, `_active_goal_id` is cleared** (before the cancel request is even sent). From then on, *any* feedback or result carrying that goal ID is logged as `[stale]` and ignored, including the confirmation of its own cancellation.
- Every feedback message and result is checked against the active goal ID by **UUID**, not by arrival order or timing.
- A goal that is accepted by the server after it has already been superseded (its send token no longer matches the pending token) is cancelled on arrival.
- AI results are versioned by sequence number, so a stale *AI* result can't start a goal either.
- **Proof (Scenario 5):** with `late_result_delay_s=1.5`, the robot delivers a "zombie" feedback message and the final result of the superseded Goal A about 1.65 s after Goal B started. The commander logs `[stale] dropped feedback from goal 1edce4d2 (active=10fa13ba)` and `[stale] ignored late result ... -> active goal 10fa13ba unaffected`, and B completes normally.

---

## Design decisions & limitations

- **Mock robot instead of Gazebo:** the assignment grades timing, arbitration and safety behaviour. A deterministic kinematic mock makes every scenario reproducible and the logs precise.
- **Human recordings instead of TTS:** synthetic espeak audio was misheard by Whisper Tiny ("90 degrees" → "the light to the breeze"). The pipeline correctly **rejected** those commands (missing angle), which is itself a useful safety demonstration, but real recordings were used for the scenarios.
- **Threads vs processes for AI:** the AI runs in threads inside the commander process. That is sufficient here because the heavy work releases the GIL and safety lives in a separate process. On weaker hardware, I would move the AI into its own process (see the memo below).
- Telemetry is published as a readable `std_msgs/String` to keep the logs self-explanatory. A real system would use `nav_msgs/Odometry`.
- Not implemented: a commander heartbeat or deadman switch on the robot side, and velocity ramps. Both are natural next steps.

---

## Edge deployment memo: NVIDIA Jetson Orin Nano

The main challenge on an Orin Nano (8 GB) is that CPU and GPU share the same unified memory. Phi-3.5 Mini at 4-bit (~2.5–3 GB with KV cache), Whisper, ROS 2, DDS and the OS all compete for it, and an out-of-memory event could take down the robot processes along with the AI. I would run the LLM with a Jetson-optimised 4-bit runtime (llama.cpp with CUDA, or TensorRT-LLM/MLC), cap the context length, keep a smaller fallback model (around 1.5B parameters), and keep Whisper Tiny on the CPU in int8 so it doesn't contend with the LLM for the GPU. Inference will likely take several seconds per command, so the asynchronous design here becomes essential rather than optional. The STOP fast-path and the operator cancel must never wait on the LLM. For executor responsiveness and isolation, I would run the AI in a **separate process** (or container) with cgroup memory and CPU limits, so the kernel's OOM killer and the scheduler sacrifice the AI, not motion control. The robot and safety node would be pinned to dedicated cores (`taskset`/`isolcpus`) with real-time priority (`SCHED_FIFO` via `chrt`), and ideally written in C++. Finally, I would fix the power mode (`nvpmodel`, `jetson_clocks`), monitor thermals and memory with `tegrastats`, and treat the software watchdog as a second layer behind a hardware e-stop or microcontroller-level interlock.

---

*Author: Antonio Lopez*
