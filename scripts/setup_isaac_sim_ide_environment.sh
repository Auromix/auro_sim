#!/bin/bash
# set -x
# set -e
# Herman Ye@Auromix
# 2024-06-25

# Install pip
sudo apt install pip -y
# Install commentjson
pip3 install commentjson

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
download_url="https://raw.githubusercontent.com/Auromix/auro_sim/main/scripts"
script_name="setup_isaac_sim_ide_environment.py"
directory_name="/tmp"

# download script
wget -O "${directory_name}/${script_name}" "${download_url}/${script_name}"

# execute script
full_script_path="${directory_name}/${script_name}"
chmod +x "${full_script_path}"
python3 "${full_script_path}"
