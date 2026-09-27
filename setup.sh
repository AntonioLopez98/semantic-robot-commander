#!/usr/bin/env bash
# One-command setup for Ubuntu 24.04 + ROS 2 Jazzy (ROS must already be installed).
# Usage, from the repository root:  ./setup.sh
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy not found in /opt/ros/jazzy."
  echo "Install it first: https://docs.ros.org/en/jazzy/Installation.html"
  exit 1
fi

set -e
echo "[1/4] System packages"
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions curl zstd

echo "[2/4] Python packages"
pip install --user --break-system-packages -r "$REPO/requirements.txt"

echo "[3/4] Ollama + Phi-3.5 Mini"
if ! command -v ollama >/dev/null 2>&1; then
  curl -fsSL https://ollama.com/install.sh | sh
fi
if ! curl -s http://localhost:11434 >/dev/null 2>&1; then
  echo "Starting Ollama server in the background..."
  (ollama serve > /tmp/ollama.log 2>&1 &)
  sleep 5
fi
ollama pull phi3.5

echo "[4/4] Building the ROS 2 workspace"
set +e
source /opt/ros/jazzy/setup.bash
set -e
cd "$REPO"
colcon build --symlink-install

echo
echo "Setup complete. Next:"
echo "  source $REPO/install/setup.bash"
echo "  cd $REPO/scenarios && ./run_scenario.sh 1   # scenarios 1-5"