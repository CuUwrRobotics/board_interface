# @Author: Nick Steele <nichlock>
# @Date:   21:40 Sep 18 2020
# @Last modified by:   Nick Steele
# @Last modified time: 12:46 Nov 26 2020

echo Copying files...

# Copy your packages here
cp -r $temporary_package_directory $final_package_directory/board_interface

# Copy our catkin Makefile
cp -r $temporary_package_directory/catkin-setups/Makefile $final_package_directory/../Makefile
