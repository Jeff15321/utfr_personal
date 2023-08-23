# dv24

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

### Launch
```
ros2 launch <node name> <launch file name>
```
