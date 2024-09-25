#!/bin/bash
# set -x
set -e
# Herman Ye@Auromix
# 2024-09-02

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Prompt user to input Isaac Sim version
read -p "Please enter the Isaac Sim version you want to configure (e.g., 4.2.0): " input_version

# Validate the input version
if [[ ! $input_version =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Invalid version format. Please use the format x.x.x (e.g., 4.2.0)."
    exit 1
fi

ISAAC_SIM_VERSION="isaac-sim-${input_version}"
ISAAC_SIM_PATH="${HOME}/.local/share/ov/pkg/${ISAAC_SIM_VERSION}"
ISAAC_SIM_PYTHON_SCRIPT="${ISAAC_SIM_PATH}/python.sh"
ISAAC_SIM_PYTHON_EXE="${ISAAC_SIM_PATH}/kit/python/bin/python3"
ISAAC_SIM_APP_SCRIPT="${ISAAC_SIM_PATH}/isaac-sim.sh"
BASHRC="${HOME}/.bashrc"

if [ ! -d "$ISAAC_SIM_PATH" ]; then
    echo "ISAAC Sim version '$ISAAC_SIM_VERSION' is not installed."
    echo "Please download and install the corresponding version."
    exit 1
fi

START_MARKER="# Isaac Sim Environment Config [Start]"
END_MARKER="# Isaac Sim Environment Config [End]"

if grep -qF "$START_MARKER" "$BASHRC"; then
    echo "Existing Isaac Sim configuration found in ~/.bashrc. Skipping configuration."
else
    # Backup bashrc
    cp ~/.bashrc ~/.bashrc.bak
    echo "Bashrc backup created at ~/.bashrc.bak"
    # Add new configurations
    {
        echo ""
        echo $START_MARKER
        echo "export ISAAC_SIM_VERSION=$ISAAC_SIM_VERSION"
        echo "export ISAAC_SIM_PATH=${ISAAC_SIM_PATH}"
        echo "export ISAAC_SIM_PYTHON_SCRIPT=${ISAAC_SIM_PYTHON_SCRIPT}"
        echo "export ISAAC_SIM_PYTHON_EXE=${ISAAC_SIM_PYTHON_EXE}"
        echo "export ISAAC_SIM_APP_SCRIPT=${ISAAC_SIM_APP_SCRIPT}"
        echo "alias omni_python=${ISAAC_SIM_PYTHON_SCRIPT}"
        echo "alias isaac_sim=${ISAAC_SIM_APP_SCRIPT}"
        echo $END_MARKER
        echo ""
    } >>"$BASHRC"
fi

echo ""
echo "Isaac Sim environment configured successfully."
echo "Please run omni_python <your_python_script_for_isaac_sim> to use Isaac Sim with Python."
echo "Please run isaac_sim to use Isaac Sim GUI."
