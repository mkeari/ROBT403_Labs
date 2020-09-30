## Installation
In order to run the code, 
1) Copy and paste the packages to your catkin workspace. And stay in this directory.
2) Source the setup.bash
```
source devel/setup.bash
```
3) Compile the packages
```
catkin_make
```
4) Run Gazebo
```
roslaunch gazebo_robot merey_gazebo.launch
```
5) Run Rviz
```
roslaunch robot_lab4 moveit_planning_execution.launch
```
6) Run the selected code by corresponding executable (commands are provided below) 

### Task 2 - Movement by x-axis
Executable:
```
rosrun lab4_test line_moveit
```
Video: https://youtu.be/GPGJtaTHAQY

### Task 3 - Drawing a rectangle
Executable:
```
rosrun lab4_test rectangle_moveit
```
Video: https://youtu.be/y2VU174i4As

### Task 4 - Drawing a circle
Executable:
```
rosrun lab4_test circle_moveit
```
Video: https://youtu.be/RkA03gqJpNE
