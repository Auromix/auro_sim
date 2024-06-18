#!/bin/bash
# set -x
set -e
# Herman Ye@Auromix
# 2024-06-18

# Create final package in /Download
echo "Create final package in /Download"
mkdir -p ~/Downloads/isaac_sim_assets_final_package
# Copy package 1 to final_package
echo "Copy package 1 to final_package"
sleep 3
cp -v -r ~/Downloads/isaac-sim-assets-1-4.0.0/* ~/Downloads/isaac_sim_assets_final_package/
# Copy package 2 to final_package
echo "Copy package 2 to final_package"
sleep 3
cp -v -r ~/Downloads/isaac-sim-assets-2-4.0.0/* ~/Downloads/isaac_sim_assets_final_package/
# Copy package 3 to final_package
echo "Copy package 3 to final_package"
sleep 3
cp -v -r ~/Downloads/isaac-sim-assets-3-4.0.0/* ~/Downloads/isaac_sim_assets_final_package/
# Copy package 4 to final_package
echo "Copy package 4 to final_package"
sleep 3
cp -v -r ~/Downloads/isaac-sim-assets-4-4.0.0/* ~/Downloads/isaac_sim_assets_final_package/

echo "All assets are copied to final_package"
