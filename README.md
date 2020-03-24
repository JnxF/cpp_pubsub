# Minimum publisher/subscriber

```bash
colcon build --packages-select cpp_pubsub
. install/setup.bash
ros2 run cpp_pubsub talker
ros2 run cpp_pubsub listener
```


