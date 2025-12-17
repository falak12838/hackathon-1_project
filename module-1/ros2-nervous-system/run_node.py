#!/usr/bin/env python3

"""
Script to run the ROS2 nervous system node
This is a demonstration script for running the nervous system node
"""

import subprocess
import sys

def run_ros2_node():
    """Function to run the ROS2 nervous system node"""
    try:
        # Check if ROS2 is available
        result = subprocess.run(['which', 'ros2'], capture_output=True, text=True)
        if result.returncode != 0:
            print("ROS2 is not installed or not in PATH")
            print("Please install ROS2 Humble Hawksbill or later version")
            return False

        print("Starting ROS2 Nervous System Node...")
        print("To stop the node, press Ctrl+C")

        # Run the ROS2 node
        subprocess.run(['ros2', 'run', 'ros2_nervous_system', 'nervous_system_node'])

    except KeyboardInterrupt:
        print("\nShutting down the nervous system node...")
    except FileNotFoundError:
        print("ROS2 is not installed or not in PATH")
        print("Please install ROS2 Humble Hawksbill or later version")
        return False

    return True

if __name__ == "__main__":
    print("ROS2 Nervous System Package Runner")
    print("===================================")

    success = run_ros2_node()

    if success:
        print("Node executed successfully")
    else:
        print("Failed to run the node")
        sys.exit(1)