#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023-2024 Herman Ye @Auromix
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Herman Ye @Auromix
# Description: Config isaac sim vscode ide environment in your project

import os

import commentjson

isaac_sim_prefix="isaac-sim-"
isaac_sim_version = input("Please enter the Isaac Sim version (e.g., 4.2.0): ").strip()
isaac_sim_version = isaac_sim_prefix + isaac_sim_version
isaac_sim_path = os.path.join(os.getenv("HOME"), ".local/share/ov/pkg", isaac_sim_version)
isaac_sim_vscode_settings_file = os.path.join(isaac_sim_path, '.vscode', 'settings.json')
current_dir = os.getcwd()
target_dir = os.path.join(current_dir, '.vscode')
target_vscode_settings_file = os.path.join(target_dir, 'settings.json')


def main():
    # Get isaac_sim vscode python analysis extraPaths
    python_analysis_extra_paths = 'python.analysis.extraPaths'
    if not os.path.exists(isaac_sim_vscode_settings_file):
        print(
            f'isaac_sim vscode python analysis extraPaths not found in {isaac_sim_vscode_settings_file}'
        )
        exit(1)

    with open(isaac_sim_vscode_settings_file, 'r', encoding='utf-8') as file:
        isaac_sim_vscode_settings = commentjson.load(file)

    extra_paths = isaac_sim_vscode_settings.get(python_analysis_extra_paths, [])
    # Add isaac_sim_path prefix to extra_paths
    extra_paths = [
        os.path.join(
            isaac_sim_path,
            path,
        )
        for path in extra_paths
    ]

    # Create target .vscode/settings.json if not exists
    if not os.path.exists(target_dir):
        print(f"The .vscode folder does not exist in {current_dir}.")
        confirm = input("Is this the root directory of your project? (y/n): ").strip().lower()
        if confirm == 'y':
            os.makedirs(target_dir, exist_ok=True)
            target_settings = {}
            with open(target_vscode_settings_file, 'w', encoding='utf-8') as file:
                commentjson.dump(target_settings, file)
        else:
            print("Operation cancelled.")
            print("Please make sure your terminal is in the root directory of your project.")
            exit(1)

    # Create settings.json if not exists
    if not os.path.exists(target_vscode_settings_file):
        with open(target_vscode_settings_file, 'w', encoding='utf-8') as file:
            commentjson.dump({}, file)

    # Read existing target .vscode/settings.json
    with open(target_vscode_settings_file, 'r', encoding='utf-8') as file:
        target_settings = commentjson.load(file)

    # Check if python.analysis.extraPaths exists in target settings.json
    if python_analysis_extra_paths in target_settings:
        # If it exists, append the extra paths to it
        target_settings[python_analysis_extra_paths].extend(extra_paths)
    else:
        # If it doesn't exist, create it with the extra paths
        target_settings[python_analysis_extra_paths] = extra_paths

    # Delete duplicate paths
    target_settings[python_analysis_extra_paths] = list(
        set(target_settings[python_analysis_extra_paths])
    )

    # Write updated target settings.json
    with open(target_vscode_settings_file, 'w', encoding='utf-8') as file:
        commentjson.dump(target_settings, file, indent=4, separators=(',', ': '))
    print(f'Updated {target_vscode_settings_file} with extraPaths.')


if __name__ == "__main__":
    main()
