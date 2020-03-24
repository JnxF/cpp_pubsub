# Minimum publisher/subscriber

To build the package and execute nodes independently

```bash
colcon build --packages-select cpp_pubsub
. install/setup.bash
ros2 run cpp_pubsub talker
ros2 run cpp_pubsub listener
```

To run launch files:
```
ros2 launch both.py
ros2 launch talker.py
ros2 launch listener.py

```
