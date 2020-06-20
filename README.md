# The cpp_pubsub package

This package includes:

- Source code:
	- `publisher_member_function.cpp` - a C++ publisher to the `topic` ROS topic
	- `subscriber_member_function.cpp` - a C++ listener to the `topic` ROS topic
	- `cpp_fuzztarget.cpp` - A publisher that publishes the entire standard
	  input as a message.

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
├── afl-config.bash
├── launch
│   ├── both.launch.py
│   ├── listener.launch.py
│   └── talker.launch.py
├── package.xml
├── src
│   ├── cpp_fuzztarget.cpp
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
source start.bash
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

To run the fuzz target:
```
cd /opt/ros_ws/install/cpp_pubsub/lib/cpp_pubsub/
echo "line 1\nline2" | ./cpp_fuzztarget
```

Command to run the fuzztalker and a listener:
```bash
python3 fuzztarget.py & ros2 run cpp_pubsub listener
```

If needed multiple docker images:

```bash
docker ps # Check running docker instance
docker exec -it d4c696afe176 bash # Replace by your existing id
# Now inside, rerun this command and you'll operate normaly
source /opt/ros/eloquent/setup.bash
```


## Running AFL
```
source src/cpp_pubsub/afl-config.bash  
colcon build --packages-select cpp_pubsub
cd install/cpp_pubsub/lib/cpp_pubsub/
afl-fuzz -i inputs/ -o outputs/ ./cpp_fuzztarget 
```

# Starting the fuzzer
```
source src/cpp_pubsub/afl-config.bash  
colcon build --packages-select cpp_pubsub
. install/setup.bash
ros2 run cpp_pubsub listener &
cd install/cpp_pubsub/lib/cpp_pubsub/
afl-fuzz -m none -t 15000 -i inputs/ -o outputs/ ./cpp_fuzztarget
```

# Injecting code
```
colcon build --packages-select cpp_pubsub
. install/setup.bash
cd install/cpp_pubsub/lib/cpp_pubsub/
LD_PRELOAD=./libinjector.so ./listener
```

# AFL with injected code
```
source src/cpp_pubsub/afl-config.bash  
colcon build --packages-select cpp_pubsub
cd install/cpp_pubsub/lib/cpp_pubsub/
export LD_PRELOAD=./libinjector.so
afl-fuzz -m none -t 15000 -i inputs/ -o outputs/ LD_PRELOAD=./libinjector ./listener
```
LD_PRELOAD=/libinjector.so afl-fuzz -m none -t 15000 -i inputs/ -o outputs/ -- ./listener



vim input.txt

cat input.txt | LD_PRELOAD=./libinjector.so ./listener

# Fixed
```
source src/cpp_pubsub/afl-config.bash  
colcon build --packages-select cpp_pubsub
. install/setup.bash
cd install/cpp_pubsub/lib/cpp_pubsub/
export AFL_PRELOAD=/opt/ros_ws/install/cpp_pubsub/lib/cpp_pubsub/libinjector.so
afl-fuzz -m none -i inputs/ -o outputs/ ./listener


cat inputs/input0.txt | LD_PRELOAD=./libinjector.so ./listener

```

source src/cpp_pubsub/afl-config.bash  
colcon build --packages-select cpp_pubsub
. install/setup.bash
cd install/cpp_pubsub/lib/cpp_pubsub/
time python3 ../../../../src/cpp_pubsub/generators/gen.py | LD_PRELOAD=./libinjector.so ./listener



colcon build --packages-select cpp_pubsub
. install/setup.bash
cd install/cpp_pubsub/lib/cpp_pubsub/
time cat /dev/urandom | LD_PRELOAD=./libinjector.so ./listener



afl-fuzz -m none -t 15000 -i inputs/ -o outputs/ ./listener


afl-fuzz -d -i inputs/ -o outputs/ -N tcp://127.0.0.1/7413 -P RTSP -D 100000 -n -q 3 -s 3 -E -K -R ./listener 7413


if [ $? -eq 0 ]; then
    echo OK
else
    echo FAIL
fi


git clone https://github.com/linux-test-project/lcov.git
cd lcov
make install
cd ..

 (echo "as das das das das das da sd asd " && sleep 2 && echo "holita" )| LD_PRELOAD=./libinjector.so ./listener