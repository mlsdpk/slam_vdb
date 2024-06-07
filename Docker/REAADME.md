# Docker Instructions

## Build Docker Image

To build the docker image

```bash
docker build --tag YOUR_IMAGE_NAME:YOUR_TAG -f DockerFile.jazzy .  
```

## Run the docker

Mount the slam_vdb source code to the docker container and attach the display for visualization

```bash
docker run -it -v SLAM_VDB_LOCATION:/slam_vdb_ws/slam_vdb -e DISPLAY=docker.for.mac.host.internal:0 YOUR_IMAGE_NAME:YOUR_TAG bash
```
