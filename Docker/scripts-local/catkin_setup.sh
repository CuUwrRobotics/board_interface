echo Starting catkin workspace setup

cd ~/
# mkdir catkin_ws
mkdir catkin_ws/src
cd catkin_ws

echo Running catkin_make in workspace...
source /opt/ros/melodic/setup.sh
catkin_make clean >> /dev/null


echo Cloning watchdog into workspace...
cd src
git clone --branch master https://github.com/CuUwrRobotics/watchdog

echo Cleaning workspace...
cd catkin_ws
catkin_make clean >> /dev/null

echo Finished catkin workspace setup
