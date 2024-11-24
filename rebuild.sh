#!/bin/bash

# Function to show usage
show_usage() {
    echo "Usage: ./rebuild.sh [package_name] [launch_file]"
    echo "Example: ./rebuild.sh armbot_description display.launch.py"
    echo "If no package is specified, rebuilds all packages"
}

# Get package name and launch file from arguments
PACKAGE=$1
LAUNCH_FILE=$2

# Clear terminal
clear

echo "🔨 Building package(s)..."
if [ -z "$PACKAGE" ]; then
    # Build all packages if no package specified
    colcon build --symlink-install
else
    # Build specific package
    colcon build --symlink-install --packages-select $PACKAGE
fi

# Source the workspace
echo "🔄 Sourcing workspace..."
source install/setup.bash

# Launch the file if specified
if [ ! -z "$LAUNCH_FILE" ]; then
    echo "🚀 Launching $LAUNCH_FILE..."
    if [ ! -z "$PACKAGE" ]; then
        ros2 launch $PACKAGE $LAUNCH_FILE
    else
        echo "⚠️ Please specify a package name to launch a file"
        show_usage
    fi
fi