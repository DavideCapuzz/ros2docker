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
  cp template/install_robot_deps.sh ./tmp/install_robot_deps.sh
  echo "File install_robot_deps not found, using the template."
fi

if [ -f ../endfunction.sh ]; then
  # Copy the file from the parent directory to ./tmp
  cp ../endfunction.sh ./tmp/endfunction.sh
  echo "File copied successfully."
else
  # If the file is not found, use the template instead
  cp template/endfunction.sh ./tmp/endfunction.sh
  echo "File endfunction not found, using the template."
fi