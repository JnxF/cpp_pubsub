# The cpp_pubsub package

This package includes:

- Source code:
	- `publisher_member_function.cpp` - a C++ publisher to the `topic` ROS topic
	- `subscriber_member_function.cpp` - a C++ listener to the `topic` ROS topic

- Multiple launch files:
	- `talker.launch.py` - starts a publisher 
	- `listener.launch.py` - starts a listener
	- `both.launch.py` - starts both a publisher and a listener

- Configuration files:
	- `package.xml` - with information of the package and dependencies (just `rclcpp`and `std_msgs`).
	- `CMakeLists.txt` - defines entry points `talker` and `listener` for C++ files.

```
.
├── CMakeLists.txt
├── Dockerfile
├── README.md
├── launch
│   ├── both.launch.py
│   ├── listener.launch.py
│   └── talker.launch.py
├── package.xml
├── src
│   ├── fuzztarget.py
│   ├── publisher_member_function.cpp
│   └── subscriber_member_function.cpp
└── start.bash
```

# Execution

The execution consists of several steps

## Run the docker container
I created a handy script to initialize `docker-machine` (step just necessary in Mac),
build a docker container from the Dockerfile and running it.

Navigate to the root path of this repository and run
```bash
chmod +x start.bash  # Just first time
./start.bash
```

Expect your console to look like the following.
```bash
root@d4c696afe176:/opt/ros_ws/# 
```

Now, to build the package and execute nodes independently, you can try:

```bash
colcon build --packages-select cpp_pubsub
. install/setup.bash
ros2 run cpp_pubsub talker
ros2 run cpp_pubsub listener
```

To run launch files:
```
ros2 launch both.launch.py
ros2 launch talker.launch.py
ros2 launch listener.launch.py
```

Command to run the fuzztalker and a listener:
```bash
python3 fuzztarget.py & ros2 run cpp_pubsub listener
```

If needed multiple docker images:

```bash
docker exec -it d4c696afe176 bash # Replace by your existing id
# Now inside, rerun this command and you´ll operate normaly
source /opt/ros/eloquent/setup.bash
```
