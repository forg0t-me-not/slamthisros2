# slamthisros2

how to start stadium

source /opt/ros/humble/setup.bash

ros2 launch ~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/course_final.launch.py

Task A (Need To run Stadium first)
  source /opt/ros/humble/setup.bash
  ros2 run final_project path_tracker
  - After run the code will automatic run track to yellow path and read QR at the end, After that thebturtle bot will turn right and wait for new ArUco (now from ccode is -1 can chance later)
    ros2 run rqt_image_view rqt_image_view
- To view the camera raw 

