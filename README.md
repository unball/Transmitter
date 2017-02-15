# Communication
Communication repository for @unball


##Building the communication node

1. If it's your first time running this code, and you use ubuntu, please run "sh fix.sh" (your arduino must be plugged in)

2. If you don't have installed rosserial and joy, follow this steps:
	
	* run `sudo apt-get install ros-<rosdistro>-rosserial-arduino`
	* run `sudo apt-get install ros-<rosdistro>-joy`


3. Make a link on your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)


##Running

Run `roslaunch communication_node run_communication.launch`


##Launching the whole program with the joystick


`cd Communication`

* Terminal 1: `sudo xboxdrv --silent`
* Terminal 2: `roslaunch communication run_joystick.launch`
* Terminal 3: `python position_control.py`
* Terminal 4: `python differential_model.py`
* Terminal 5: `roslaunch communication run_communication.launch`
* Terminal 6: `python dummy_pub.py` 
* Terminal 7: `rostopic echo/joy`
* Terminal 8: `rostopic echo/target_positions_topic`
* Terminal 9: `rostopic echo/robots_speeds`
* Terminal 10: `rostopic echo/wheels_speeds_msg`
* Terminal 11: `rostopic echo/comm_msg`

