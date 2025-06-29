#!/bin/bash

SCRIPT_PATH=$(dirname $(realpath "$0"))

PARENT_PATH=$(dirname "$SCRIPT_PATH")

# Function to build the Docker image
build_docker_image()
{
    LOG="Building Docker image yoyo:latest ..."
    print_debug

    # Build the Docker image
    sudo docker image build -f $SCRIPT_PATH/Dockerfile -t yoyo:latest $PARENT_PATH
}

# Function to create a shared folder
create_shared_folder()
{
    # Check if the directory doesn't exist
    if [ ! -d "$HOME/automaticaddison/shared/ros2" ]; then
        # Set a log message for folder creation
        LOG="Creating $HOME/automaticaddison/shared/ros2 ..."

        # Print the log message
        print_debug
        mkdir -p $HOME/automaticaddison/shared/ros2
    fi
}

# Function to print debug messages
print_debug()
{
    # Print an empty line for readability
    echo ""

    echo $LOG

    echo ""
}

# Main execution flow

# First, create the shared folder that will be mounted in the container
create_shared_folder

# Then, build the Docker image
build_docker_image