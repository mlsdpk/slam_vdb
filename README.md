# Slam VDB

## Build

Let's first build the `slam_vdb` library

``` bash
cd slam_vdb
mkdir build && cd build
cmake ..
make install
```

This will install the library into the GNU default location which is usually `/usr/local`. If you have chosen to install at your own specific location, you can use the CMAKE option `-DSLAMVDB_INSTALL_DIR`.

Next, let's build the ros layer `slam_vdb_ros`

If you had installed the slam_vdb using default, you can simply build the slam_vdb_ros using the following command.

```bash
cd slam_vdb_ros
colcon build --symlink-install
```

In case, if you had installed slam_vdb to particular location, you need to pass the location of the slam_vdb library as make arg to the slam_vdb_ros.

```bash
colcon build --ament-cmake-args -DSLAM_VDB_LOCATION="YOUR_LOCATION"
```
