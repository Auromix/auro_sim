#!/bin/bash
# set -x
set -e
# Herman Ye@Auromix
# 2024-06-25

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# echo script_dir: $script_dir

# Config isaac sim environment
ISAAC_SIM_VERSION="isaac-sim-4.0.0"
ISAAC_SIM_PATH="${HOME}/.local/share/ov/pkg/${ISAAC_SIM_VERSION}"
ISAAC_SIM_IDE_ENV_SETUP_OFFICIAL_DEFAULT_SCRIPT="${ISAAC_SIM_PATH}/setup_python_env.sh"

# Extend the default setup script
EXTENSIONS_DIRS=(
    "$ISAAC_SIM_PATH/exts"
    "$ISAAC_SIM_PATH/extsPhysics"
    "$ISAAC_SIM_PATH/extscache"
    "$ISAAC_SIM_PATH/kit/exts"
    "$ISAAC_SIM_PATH/kit/extscore"
    "$ISAAC_SIM_PATH/kit/kernel/py"
    # "$ISAAC_SIM_PATH/kit/plugins/bindings-python"
    "$ISAAC_SIM_PATH/kit/python/lib/python3.10/site-packages"
    "$ISAAC_SIM_PATH/python_packages"
)

# Setup the environment
source ${ISAAC_SIM_IDE_ENV_SETUP_OFFICIAL_DEFAULT_SCRIPT}

for ext_dir in "${EXTENSIONS_DIRS[@]}"; do
    # Check if the directory exists and is a directory
    if [ ! -d "$ext_dir" ]; then
        echo "Warning: $ext_dir does not exist or is not a directory. Skipping addition to PYTHONPATH."
        continue
    fi

    # Get the list of directories in the extension directory
    for dir in "$ext_dir"/*/; do
        # Get the name of the directory
        # echo $(basename "$dir")
        dir_name=$(basename "$dir")
        # Add to PYTHONPATH
        export PYTHONPATH="$PYTHONPATH:$dir"
    done
done
