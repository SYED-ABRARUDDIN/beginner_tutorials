# ROS 2 Beginner Tutorials - ENPM808X Week-9

Services on Publisher subscriber package for the ROS2 beginner tutorials 
**Abraruddin Syed** (120109997) 


### Building the Code

```bash
$ echo "source /opt/ros/humble/setup.bash"
# Go your ros2 workspace
$ cd ~/ros2_ws/src
#Clone the repository
$ git clone https://github.com/SYED-ABRARUDDIN/beginner_tutorials.git
#Go back to the ws directory
$ cd ~/ros2_ws
# Run the publisher in terminal#1
ros2 run beginner_tutorials talker
# Run the subscriber in terminal#2 (Split the terminal and source ROS2 and the workspace setup.bash)
ros2 run beginner_tutorials listener 
```

### Launch Files and checking service
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Run the publisher in terminal#1
$ ros2 launch beginner_tutorials service_launch.py publish_freq:=500
# Run the subscriber in terminal#2 (Split the terminal and source ROS2 and the workspace setup.bash)
ros2 run beginner_tutorials listener
#Open  terminal#3 and type the following
$ ros2 service call /modify_string beginner_tutorials/srv/ModifyString  "{input_string: 'Abrar'}"
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
