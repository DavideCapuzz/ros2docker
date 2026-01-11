#!/bin/bash

# Check if the destination directory exists, create it if it doesn't
mkdir -p ./tmp
# Check if the file exists in the parent directory
if [ -f ../docker-compose.yml ]; then
  # Copy the file from the parent directory to ./tmp
  cp ../docker-compose.yml ./tmp/docker-compose.yml
  echo "File copied successfully."
elif [ -f ../setup.ini ]; then
  bash scripts/init_docker-compose.sh ../setup.ini
  echo "File install_robot_deps not found, using the template."
else
  # If the file is not found, use the template instead
  cp template/docker-compose.yml ./tmp/docker-compose.yml
  echo "File install_robot_deps not found, using the template."
fi

if [ -f ../entrypoint.sh ]; then
  # Copy the file from the parent directory to ./tmp
  cp ../entrypoint.sh ./tmp/entrypoint.sh
  echo "File copied successfully."
else
  # If the file is not found, use the template instead
  cp template/entrypoint.sh ./tmp/entrypoint.sh
  echo "File install_robot_deps not found, using the template."
fi

if [ -f ../install_robot_deps.sh ]; then
  # Copy the file from the parent directory to ./tmp
  cp ../install_robot_deps.sh ./tmp/gazebo/install_robot_deps.sh
  cp ../install_robot_deps.sh ./tmp/robot1/install_robot_deps.sh
  echo "File copied successfully."
else
  # If the file is not found, use the template instead
  cp template/install_robot_deps.sh ./tmp/gazebo/install_robot_deps.sh
  cp template/install_robot_deps.sh ./tmp/robot1/install_robot_deps.sh
  echo "File install_robot_deps not found, using the template."
fi

if [ -f ../endfunction.sh ]; then
  # Copy the file from the parent directory to ./tmp
  cp ../endfunction.sh ./tmp/gazebo/endfunction.sh
  cp ../endfunction.sh ./tmp/robot1/endfunction.sh
  echo "File copied successfully."
else
  # If the file is not found, use the template instead
  cp template/endfunction.sh ./tmp/gazebo/endfunction.sh
  cp template/endfunction.sh ./tmp/robot1/endfunction.sh
  echo "File endfunction not found, using the template."
fi