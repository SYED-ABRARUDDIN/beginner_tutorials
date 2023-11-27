# ROS 2 Beginner Tutorials - ENPM808X Week-9

Services on Publisher subscriber package for the ROS2 beginner tutorials 
tf2, ros2_bag, unittesting are aslo included 
**Abraruddin Syed** (120109997) 


### Building the Code

```bash
$ echo "source /opt/ros/humble/setup.bash"
# Go your ros2 workspace
$ cd ~/ros2_ws/src
#Clone the repository
$ git clone https://github.com/SYED-ABRARUDDIN/beginner_tutorials.git
#Go back to the ws directory
$ source install/setup.bash
# Run the publisher in terminal#1
ros2 run beginner_tutorials talker
# Run the subscriber in terminal#2 (Split the terminal and source ROS2 and the workspace setup.bash)
ros2 run beginner_tutorials listener 
```

### Launch Files and checking service
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source install/setup.bash
# Run the publisher in terminal#1
$ ros2 launch beginner_tutorials service_launch.py publish_frequency:=500
# Run the subscriber in terminal#2 (Split the terminal and source ROS2 and the workspace setup.bash)
ros2 run beginner_tutorials listener
#Open  terminal#3 and type the following
$ ros2 service call /modify_string beginner_tutorials/srv/ModifyString  "{input_string: 'Abrar'}"
```



### TF Frames

The talker node in this package now publishes a static tf transform between 2 frames, `world` and `talk` as a arbitrary transform. To run the publisher, run
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Run the publisher in terminal
$ ros2 run beginner_tutorials talker
```
To view the tf transform, run the following commands in a separate terminal
```bash
 # In a new terminal window, echo the topic that broadcasts the static frame:
$ ros2 topic echo /tf_static
# In a new terminal window, get more information about the frames
$ ros2 run tf2_tools view_frames
```

### Testing
To run the unit tests and verify their working, run the commands below.
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
# Build the package:
$  colcon build --packages-select beginner_tutorials
# Install the package:
$  source install/setup.bash
# Run the unit tests:
$ colcon test --packages-select beginner_tutorials
# View the results pf the tests:
$ cat log/latest_test/beginner_tutorials/stdout_stderr.log
```

### ROS2 Bag Functionality
This package supports recording and playback of ros2 bags. The launch file has been modified to support ros2 bag recording. To record use the `ros2_bag_start` parameter (True/False).

```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Run the launch file in terminal with the ros2_bag_start parameter as true
$ ros2 launch beginner_tutorials talkernode.launch.py ros2_bag_start:=True
```
The above ros2 bag  can be found in the workspace directory where the command was run.
To inspect and playback the ros2 bag.
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Inspect the ros2 bag
$  ros2 bag info <bag_name>
# Play back the contents of the ros2 bag
$  ros2 bag play <bag_name>
```
To check the working, in a seperate terminal run
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Run the listener in terminal
$ ros2 run beginner_tutorials listener
```

### Check style guidelines
```bash
#In the package directory
cd ~/ros_ws/src/beginner_tutorials

# Cppcheck
$ cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" ) --check-config > results/cppcheck.txt

# cpplint
$ cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order  src/*.cpp >  results/cpplint.txt
```

### Buidling Doxygen Documentation
```bash
$ cd ~/ros_ws
#Run the colcon build on the doxygen docs cmake target
$ colcon build --packages-select beginner_tutorials --cmake-target docs