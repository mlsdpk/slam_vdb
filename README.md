# Slam VDB

## Build

Let's first build the `slam_vdb` library

``` bash
cd slam_vdb
mkdir build && cd build
cmake -DSLAMVDB_INSTALL_DIR="../install" ..
make install
```

Next, let's build the ros layer `slam_vdb_ros`

```bash
cd slam_vdb_ros
colcon build --symlink-install
```
