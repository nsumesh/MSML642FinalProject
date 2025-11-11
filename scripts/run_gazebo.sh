#!/bin/bash
source $(dirname "$0")/setup_env.sh

WORLD_PATH="$(dirname "$0")/../gazebo_worlds/small_warehouse.world"

# --- Launch Gazebo depending on GUI availability ---
if [ -z "$DISPLAY" ]; then
    echo "No display detected — running Gazebo headless..."
    gzserver --verbose "$WORLD_PATH"
else
    echo "Launching Gazebo with GUI..."
    gazebo --verbose "$WORLD_PATH"
fi

