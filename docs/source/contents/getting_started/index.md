# Getting Started

This guide will show you all the necessary installation steps required to use SLAM-VDB for developing and testing simultaneous localization and mapping applications.

## Installation

### Building from source

#### Step 1: Build the `slam_vdb` Library

First, clone the `slam_vdb` repository and build the library:

```sh
$ git clone https://github.com/mlsdpk/slam_vdb.git
$ cd slam_vdb
$ mkdir build && cd build
$ cmake ..
$ make install
```

This will install the library into the GNU default location, which is usually `/usr/local`. If you prefer to install it at a specific location, use the CMake option `-DSLAMVDB_INSTALL_DIR`:

```sh
$ cmake -DSLAMVDB_INSTALL_DIR=/your/installation/path ..
$ make install
```

#### Step 2: Build the ROS wrapper `slam_vdb_ros`

If you installed `slam_vdb` using the default flags, you can build the `slam_vdb_ros` wrapper as follows:

```sh
$ cd slam_vdb_ros
$ colcon build --symlink-install
```

If you installed `slam_vdb` in a custom directory, specify the location of the `slam_vdb` library when building `slam_vdb_ros`:

```sh
$ colcon build --ament-cmake-args -DSLAM_VDB_LOCATION="/path/to/slam_vdb/install_directory"
```

### Using Docker

#### Prerequisites
Ensure Docker is installed on your system. Follow the official [Docker installation guide](https://docs.docker.com/engine/install/) if it is not installed.

#### Setup

To set up Docker for running the `slam_vdb` package, follow these steps:

1. **Run the Setup Script**

   Navigate to the `Docker` directory in the main repository and execute the `setup.sh` script. This script will create a `bash`/`zsh` function on your machine inside `bashrc`/`zshrc` that will allow you to compose a Docker image and run the containers.

   ```sh
   $ cd Docker
   $ ./setup.sh
   ```

This is a one-time script. You don't need to run it every time you want to use Docker with `slam_vdb`.

#### Build and Run

Once the setup is complete, you can build and run the Docker image using the following steps:

1. **Build the Docker Image**

    Specify your Docker image name and the path to the source package:

    ```sh
    $ ros_dep YOUR_IMAGE_NAME /path/to/the/pkg
    ```

2. **Launch a New Terminal in Your Docker Container**

    To open a new terminal in your running Docker container, use the following command:

    ```sh
    $ docker exec -it YOUR_CONTAINER_NAME /bin/bash
    ```
    Note: Your image name will also be your container name.

3. **Visualize Your Docker Container**

    Once the build is complete, your containers should be up and running. You can visualize your Docker container by navigating to `http://localhost:8080/vnc.html`. This visualization is made possible with the repository [ros2-docker-dev](https://github.com/adeeb10abbas/ros2-docker-dev).

#### Customizing the Dockerfile

The current `Dockerfile` is configured for `ros:foxy`. You can modify the `Dockerfile` according to your requirements. The `Docker` folder in the main repository contains the necessary files.