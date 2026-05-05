import os
import time

# Define the 4 sides of the square using Y (left/right) and Z (up/down)
# Now lets move in a plane, then go to another plane.
# Fisrt we will move left, up, right, down, then go to x = 0.5 and repeat the same pattern.
# Then we come back to the original plane
speed = 0.5

movements = [
    #First Plane
    "linear: {x: 0.0, y: 0.3, z: 0.0}",   # Move Left
    "linear: {x: 0.0, y: 0.0, z: 0.3}",   # Move Up
    "linear: {x: 0.0, y: -0.3, z: 0.0}",  # Move Right
    "linear: {x: 0.0, y: 0.0, z: -0.3}",   # Move Down

    # Set speed to 0.0 for the next move to ensure we stop before changing direction
    "linear: {x: 0.0, y: 0.0, z: 0.0}",   # Stop before changing direction

    "linear: {x: 0.5, y: 0.0, z: 0.0}",  # Move Forward to next plane

    # Set speed to 0.0 for the next move to ensure we stop before changing direction
    "linear: {x: 0.0, y: 0.0, z: 0.0}",   # Stop before changing direction

    # Second Plane
    "linear: {x: 0.0, y: 0.5, z: 0.0}",   # Move Left
    "linear: {x: 0.0, y: 0.0, z: 0.5}",   # Move Up
    "linear: {x: 0.0, y: -0.5, z: 0.0}",  # Move Right
    "linear: {x: 0.0, y: 0.0, z: -0.5}",   # Move Down

    # Set speed to 0.0 for the next move to ensure we stop before changing direction
    "linear: {x: 0.0, y: 0.0, z: 0.0}",   # Stop before changing direction

    "linear: {x: -0.5, y: 0.0, z: 0.0}",  # Move Back to original plane

    # Set speed to 0.0 for the next move to ensure we stop before changing direction
    "linear: {x: 0.0, y: 0.0, z: 0.0}"   # Stop before changing direction
]


print("Starting Square Trajectory... Press Ctrl+C to stop.")

try:
    while True:
        for move in movements:
            # Construct the Gazebo command
            cmd = f'gz topic -t "/model/aruco_target/cmd_vel" -m gz.msgs.Twist -p "{move}, angular: {{x: 0.0, y: 0.0, z: 0.0}}"'
            
            # Send the command to Gazebo
            os.system(cmd)
            
            # Keep moving in this direction for 2 seconds before changing
            time.sleep(1.3)
            
except KeyboardInterrupt:
    # Stop the marker if you press Ctrl+C
    stop_cmd = 'gz topic -t "/model/aruco_target/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"'
    os.system(stop_cmd)
    print("\nMarker stopped.")