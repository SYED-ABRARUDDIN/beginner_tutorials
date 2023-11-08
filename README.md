# ROS 2 Beginner Tutorials - ENPM808X Week-9

Publisher subscriber package for the ROS2 beginner tutorials 
**Abraruddin Syed** (120109997) 


### Building the Code

```bash
$ echo "source /opt/ros/humble/setup.bash"
# Go your ros2 workspace
$ cd ~/ros2_ws/src
#Clone the repository
$ git clone git@github.com:vinay-lanka/beginner_tutorials.git
#Go back to the ws directory
$ cd ~/ros2_ws
# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
colcon build --packages-select beginner_tutorials
# After successfull build source the package
$ source ./install/setup.bash

# Run the publisher in terminal#1
ros2 run beginner_tutorials talker
# Run the subscriber in terminal#2 (Split the terminal and source ROS2 and the workspace setup.bash)
ros2 run beginner_tutorials listener 
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
