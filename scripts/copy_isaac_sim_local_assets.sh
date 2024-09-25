#!/bin/bash
# set -x
set -e
# Herman Ye@Auromix
# 2024-06-18

# Ask user to input the version number
read -p "Enter the version number (e.g., 4.2.0): " VERSION

# Create final package in /Downloads
echo "Create final package in /Download"
mkdir -p ~/Downloads/isaac_sim_assets_final_package

# Copy packages based on the version input
for i in {1..4}; do
  echo "Copy package $i to final_package (version: $VERSION)"
  sleep 3
  cp -v -r ~/Downloads/isaac-sim-assets-${i}-${VERSION}/* ~/Downloads/isaac_sim_assets_final_package/
done

echo "All assets are copied to final_package (version: $VERSION)"
