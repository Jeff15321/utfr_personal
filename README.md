# dv24
![example workflow](https://github.com/UTFR/dv24/actions/workflows/workflows.yaml/badge.svg)

### After installing ROS2 Humble and cloning the repo, run
> Read the comments in the script before running to make sure env is set properly
```
bash scripts/utfr_init
```

### Adding a new node/driver with deps
```
rosdep update -q --rosdistro=humble
rosdep install --from-paths src --ignore-src -y --rosdistro humble
```

### Building/launching
```
colcon build --<options>
source install/setup.bash
```
> [Colcon build options](https://colcon.readthedocs.io/en/released/reference/package-selection-arguments.html)

```
ros2 launch <node name> <launch file name>
```

### Unit testing
```
colcon test --<options>
```
