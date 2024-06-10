# Docker Instructions

To run the package in the docker, follow these steps

## Setup

Run the shell script `setup.sh`. It'll create a `bash`/`zsh` function on your machine inside `bashrc`/`zshrc` that will allow you to compose a docker image and run the containers

```bash
cd Docker
./setup.sh
```

This is one-time script. You don't need to run the script every time.

## Run

Simply run the following by specifying your docker image name and the path to the source to the package

```bash
ros_dep YOUR_IMAGE_NAME /path/to/the/pkg
```

To launch a new terminal on your docker container

```bash
docker exec -it YOUR_IMAGE_NAME /bin/bash
```

**Note: Your image name will be your container name**

Once the build is done, your containers should be up and running. Goto `http://localhost:8080/vnc.html` to visualize your docker container. The visualization to docker is made possible with this repo [here](https://github.com/adeeb10abbas/ros2-docker-dev).

## Summary

The current `Dockerfile` is only configured for `ros:jazzy`. You can modify the `Dockerfile` to your requirements.
