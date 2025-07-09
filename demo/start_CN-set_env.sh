#!/bin/bash

# Set PROJECT_PATH to the current directory
export PROJECT_PATH=$(pwd)

# Set IGN_GAZEBO_SYSTEM_PLUGIN_PATH using PROJECT_PATH
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$PROJECT_PATH/../phine-plugins/build/phine_plugins/"
# Set IGN_GAZEBO_RESOURCE_PATH (for model)using PROJECT_PATH
export IGN_GAZEBO_RESOURCE_PATH="$PROJECT_PATH/../model_folder"

# print the variables to verify
echo "PROJECT_PATH is set to: $PROJECT_PATH"
echo "IGN_GAZEBO_SYSTEM_PLUGIN_PATH is set to: $IGN_GAZEBO_SYSTEM_PLUGIN_PATH"
echo "IGN_GAZEBO_RESOURCE_PATH is set to: $IGN_GAZEBO_RESOURCE_PATH"

# Start the CoreNetwork
cd $PROJECT_PATH/oai_setup && docker-compose up -d
cd ..
