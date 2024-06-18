#!/bin/bash
# set -x
set -e
# Herman Ye@Auromix

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# echo script_dir: $script_dir

# Config isaac sim environment
ISAAC_SIM_VERSION="isaac-sim-4.0.0"
ISAAC_SIM_PATH="${HOME}/.local/share/ov/pkg/${ISAAC_SIM_VERSION}"
ISAAC_SIM_PYTHON_EXE="${ISAAC_SIM_PATH}/python.sh"
BASHRC="${HOME}/.bashrc"
# echo isaac_sim_path: $ISAAC_SIM_PATH
# echo isaac_sim_python_exe: $ISAAC_SIM_PYTHON_EXE
# echo bashrc: $BASHRC

if [ ! -d "$ISAAC_SIM_PATH" ]; then
    echo "ISAAC Sim version '$ISAAC_SIM_VERSION' is not installed."
    echo "Please download and install the corresponding version."
    exit 1
fi

START_MARKER="# Isaac Sim Environment Config [Start]"
END_MARKER="# Isaac Sim Environment Config [End]"

if grep -qF "$START_MARKER" "$BASHRC"; then
    echo "Existing Isaac Sim configuration found in ~/.bashrc."
    read -p "Do you want to overwrite it? (y/n): " choice
    if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
        sed -i "/$START_MARKER/,/$END_MARKER/d" "$BASHRC"
    else
        echo "No changes made to ~/.bashrc."
        exit 0
    fi
fi

# Add new configurations
{
    echo ""
    echo "$START_MARKER"
    echo "ISAAC_SIM_VERSION=\"$ISAAC_SIM_VERSION\""
    echo "export ISAAC_SIM_PATH=\"${ISAAC_SIM_PATH}\""
    echo "export ISAAC_SIM_PYTHON_EXE=\"${ISAAC_SIM_PYTHON_EXE}\""
    echo "alias omni_python=\"\${ISAAC_SIM_PYTHON_EXE}\""
    echo "$END_MARKER"
    echo ""
} >>"$BASHRC"

echo "Isaac Sim environment configured successfully."
echo "Please run omin_python <your_python_script_for_isaac_sim> to use Isaac Sim with Python."
