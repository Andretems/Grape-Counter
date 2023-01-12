
# CMP9767M

NAME: Temitope Bakare

ID: 26519207

# Autonomous Grapefruit Robot Detector with Obstacle Avoidance in a Vineyard
This is the software system for an agricultural robot called Bacchus, and it is used to estimate the crop output of the field in which it works. Bacchus works in a (simulated) vineyard called Gazebo. The robot can independently move across the vineyard and determine how many clusters of grapes are present. The robot is able to navigate by connecting to WayPoints (Nodes) placed at strategic spots. The robot has lidars and cameras for detecting obstacles and avoiding them, respectively.

#Installing dependencies
1. ```sudo apt-get update && sudo apt-get upgrade```
2. ```sudo apt install python-pip```
3. ```pip install numpy``` to install numpy
4. ```pip install -U scikit-learn``` to install scikit-learn

# Running the Simulation
1. Create a catkin workspace:
   - ```mkdir -p ~/grape-counter/src```
   - ```cd ~/grape-counter/```
   - ```catkin_make```
   - ```source devel/setup.bash```
   - clone/copy the repository into ```~/grape-counter/src``` ```git clone https://github.com/Andretems/Grape-Counter.git```
2. Change the directory in .bashrc to source from your folder ```source ~/grape-counter/devel/setup.bash```
3. Then go ahead to source the folder again to change directory into the folder. 
4. Launch the simulation environment:
   - ```roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small_s4_coarse```. If it doesn't work, terminate all simulator instances: ```killall -9 gzserver```
   - ```roslaunch uol_cmp9767m_tutorial topo_nav.launch```
   - In another terminal run ```rosrun uol_cmp9767m_tutorial set_topo_nav_goal.py``` to start simulation. The robot begins navigating and counting grapes when it gets to WayPoint0.
5. Alternatively,
   - RVIZ's topological map visualisation setting lets you see the waypoints in ```uol_cmp9767m_tutorial/config/topo_nav.rviz```
   - You can click the green arrows at the nodes seen in RVIZ to send topological_navigation goal to the robot on the simulator
   - in a new terminal, run ```rosrun uol_cmp9767m_tutorial counter.py``` to count the grape bunches or enter the sccript directory.
 
Link to repo: https://github.com/Andretems/Grape-Counter.git