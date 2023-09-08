# dv24
![workflow](https://github.com/UTFR/dv24/actions/workflows/workflows.yaml/badge.svg)

### VsCode setup
* Install C++, Python extensions
* Install ROS extension
* Install `ms-python.black-formatter` for python and `xaver.clang-format` for C++
* VsCode settings are configured under `.vscode/`. Some paths might need to be changed to enable intellisense.

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
