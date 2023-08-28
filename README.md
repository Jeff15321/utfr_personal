# dv24

### After installing ROS2 Humble and cloning the repo, run
> Read the comments in the script before running to make sure env is set properly
```
bash scripts/utfr_init
```

### When adding a new node/driver with deps
```
rosdep update -q --rosdistro=humble
rosdep install --from-paths src --ignore-src -y --rosdistro humble
```

### When building/launching
```
colcon build --<options>
source install/setup.bash
```
> [Colcon build options](https://colcon.readthedocs.io/en/released/reference/package-selection-arguments.html)

```
ros2 launch <node name> <launch file name>
```
