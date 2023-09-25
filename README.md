# dv24
![workflow](https://github.com/UTFR/dv24/actions/workflows/workflows.yaml/badge.svg)

### VsCode setup
* Install C++, Python extensions
* Install ROS extension
* Install `ms-python.black-formatter` for python and `xaver.clang-format` for C++
* If facing errors with `clang-format`, install with `sudo apt install clang-format`
* VsCode settings are configured under `.vscode/`. Some paths might need to be changed to enable intellisense.

### After installing ROS 2 Humble and cloning the repo, run
> [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
```
bash scripts/utfr_init
```

### Building/launching
```
colcon build --<options>
source install/setup.bash
```
> [Colcon build options](https://colcon.readthedocs.io/en/released/reference/package-selection-arguments.html)

> If build crashes, use the `--parallel-workers <# of nodes>` option to limit the number of nodes being built in parallel

```
ros2 launch <node name> <launch file name>
```

### Unit testing
```
colcon test --<options>
```

### Adding a new node/driver with deps
```
rosdep update -q --rosdistro=humble
rosdep install --from-paths src --ignore-src -y --rosdistro humble
```
