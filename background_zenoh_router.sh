#!/bin/bash
set -e -m

# Use environment variable to set Zenoh router target
if [[ -z "$ZENOH_TARGET" ]]; then
    printf "Using default Zenoh router configuration.\n"
else
    printf "Using custom Zenoh router configuration.\n"
    sed -i "s|// \"<proto>/<address>\"|\"$ZENOH_TARGET\"|" "$ROBOT_WORKSPACE/zenoh_confs/ROUTER_CONFIG.json5"
    export ZENOH_ROUTER_CONFIG_URI="$ROBOT_WORKSPACE/zenoh_confs/ROUTER_CONFIG.json5"
fi

# Make pretty rmz_zenoh log output
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
export RCUTILS_COLORIZED_OUTPUT=1

# Source overlay and start Zenoh router in background
source "$ROBOT_WORKSPACE/install/setup.bash" && ros2 run rmw_zenoh_cpp rmw_zenohd &
