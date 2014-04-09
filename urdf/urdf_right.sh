pfad=$PWD
rosdep install joint_state_publisher
rosdep install urdf_tutorial
roscd urdf_tutorial
roslaunch urdf_tutorial display.launch model:=$pfad/right_cam.urdf gui:=true
