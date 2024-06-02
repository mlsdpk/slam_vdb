SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
export ROS_DEV_CONTAINER_NAME="ros2"
export ROS_PROJECT_PATH="$SCRIPTPATH"

echo "Creating ros2 docker container named $ROS_DEV_CONTAINER_NAME..."
echo "Mounting the project directory $ROS_PROJECT_PATH to /home/ros2/ros2_ws/ in the container..."

docker-compose up -d --build