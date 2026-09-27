#!/usr/bin/env bash
# Usage: ./run_scenario.sh <1-5>
# Starts mock robot + voice commander, runs the scenario driver, saves logs.

N="$1"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export RCUTILS_COLORIZED_OUTPUT=0          # no colour codes in log files
export RCUTILS_LOGGING_BUFFERED_STREAM=1   # line-buffered logs
export PYTHONUNBUFFERED=1

LOG="$WS/logs/scenario$N"
rm -rf "$LOG"; mkdir -p "$LOG"

LATENCY=0.0; LATE_DELAY=0.0
[ "$N" = "3" ] && LATENCY=3.0
[ "$N" = "5" ] && LATE_DELAY=1.5

echo "=== Scenario $N (ai_latency_s=$LATENCY, late_result_delay_s=$LATE_DELAY) ==="
ros2 run mock_robot mock_robot_node --ros-args -p late_result_delay_s:=$LATE_DELAY \
  > "$LOG/robot.log" 2>&1 &
ROBOT_PID=$!
ros2 run voice_commander voice_commander_node --ros-args -p ai_latency_s:=$LATENCY \
  > "$LOG/commander.log" 2>&1 &
CMD_PID=$!

echo "Waiting for commander (LLM warm-up)..."
for i in $(seq 1 120); do
  grep -q "Voice commander ready" "$LOG/commander.log" 2>/dev/null && break
  sleep 0.5
done

echo "Running scenario driver..."
python3 "$WS/scenarios/driver.py" "$N" > "$LOG/driver.log" 2>&1

echo "Shutting down nodes..."
kill -INT $ROBOT_PID $CMD_PID 2>/dev/null
sleep 2
pkill -f mock_robot_node 2>/dev/null
pkill -f voice_commander_node 2>/dev/null

# Merge all logs into one timeline sorted by ROS timestamp
cat "$LOG/robot.log" "$LOG/commander.log" "$LOG/driver.log" \
  | grep '^\[' | sort -t']' -k2,2 > "$LOG/timeline.log"

echo "Logs saved in $LOG (timeline.log = merged view)"
echo "---------------- key events (telemetry hidden) ----------------"
grep -v '\[telemetry' "$LOG/timeline.log"