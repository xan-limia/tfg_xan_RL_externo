# Sample script to run a command in a Docker container
#
# Usage Example:
# ./run_docker.sh turtlebot3_base "roslaunch turtlebot3_gazebo turtlebot3_world.launch.py"

# Define Docker volumes and environment variables
DOCKER_VOLUMES="
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="$PWD/tb3_implement:/turtlebot3_ws/src/tb3_implements:rw" \
--volume="$PWD/tb3_descriptions:/turtlebot3_ws/src/turtlebot3/turtlebot3_description/urdf:rw" \
--volume="$PWD/circuitos/textures:/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_autorace/course/materials/textures:rw" \
--volume="$PWD/circuitos/material:/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_autorace/course/materials/scripts:rw" \
"
DOCKER_ENV_VARS="
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
"
DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}

# Run the command

docker run -it --privileged  --net=host --ipc=host $DOCKER_ARGS xanlimia/turtlebot3_base bash 


# perhaps: source devel/setup.bash

