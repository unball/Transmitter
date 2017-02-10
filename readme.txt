To launch the whole program with the joystick (while we do not make a single launch file)

cd Communication
Terminal 1: sudo xboxdrv --silent 								Terminal 7: rostopic echo/joy
Terminal 2: roslaunch communication run_joystick.launch			Terminal 8: rostopic echo/target_positions_topic
Terminal 3: python position_control.py 							Terminal 9: rostopic echo/robots_speeds
Terminal 4: python differential_model.py 						Terminal 10: rostopic echo/wheels_speeds_msg
Terminal 5: roslaunch communication run_communication.launch
Terminal 6: python dummy_pub.py 								Terminal 11: rostopic echo/comm_msg