# Getting Started

```{warning}
This project is currently in progress.
```

This guide will show you all the necessary installation steps required to use SLAM-VDB for developing and testing simultaneous localization and mapping applications.

## Installation

### Building from the source repository

Make sure we build the `slam_vdb` library first:

``` sh
$ cd slam_vdb
$ mkdir build && cd build
$ cmake ..
$ make install
```

This will install the library into the GNU default location which is usually `/usr/local`. If you have chosen to install at your own specific location, you can use the CMAKE option `-DSLAMVDB_INSTALL_DIR`.

Next, follow the instructions below to build the ros wrapper `slam_vdb_ros`.

If you had installed the `slam_vdb` using default flags, you can simply build the `slam_vdb_ros` using the following command.

```sh
$ cd slam_vdb_ros
$ colcon build --symlink-install
```

In case, if you had installed `slam_vdb` to particular location, you need to pass the location of the slam_vdb library as make arg to the `slam_vdb_ros`.

```sh
$ colcon build --ament-cmake-args -DSLAM_VDB_LOCATION="<PATH-TO-SLAM-VDB-INSTALL-DIRECTORY>"
```