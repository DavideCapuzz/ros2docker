#!/bin/bash

# Check if the destination directory exists, create it if it doesn't
mkdir -p ./tmp
# Check if the file exists in the parent directory
if [ -f ../install_robot_deps.sh ]; then
  # Copy the file from the parent directory to ./tmp
  cp ../install_robot_deps.sh ./tmp/install_robot_deps.sh
  echo "File copied successfully."
else
  # If the file is not found, use the template instead
  cp install_robot_deps_template.sh ./tmp/install_robot_deps.sh
  echo "File not found, using the template."
fi