# ariac_group-2
## Team Members
1. Aryan Mishra- 120106521
2. Gautam Sreenarayanan Nair - 119543092
3. Hitesh Kyatham - 120275364
4. Keyur Borad - 120426049
5. Sarin Ann Mathew - 119390382
**************************************
### Installing dependencies
```
sudo apt install python3-rosdep
sudo apt install openjdk-17-jdk
sudo rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths src -y --ignore-src
```
**************************************
### Instructions to run
#### 1. Run the following command in terminal-1
```
ros2 launch ariac_gazebo ariac.launch.py trial_name:=final_spring2024
```
#### 2. Run the following command in terminal-2 (after terminal-1 logs "You can now start the competition!")
```
ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py
```
#### 3. Run the following command in terminal-3 (after move-it is launched properly)
```
ros2 launch final_group_2 final.launch.py
```
**************************************
### Python Libraries

YOLO V8 MODEL
```
pip install ultralytics
```
OPEN CV - CONTRIB
```
pip install opencv-contrib-python
```
OPEN CV
```
pip install opencv-python
```
   
