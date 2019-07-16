# ROS
the 2 files in cartographer_ros should be copy to :

	cartographer_ros/cartographer_ros/configuration_files/2dLidar_mapping.lua
  
	cartographer_ros/cartographer_ros/launch/2dLidar_mapping.
  
  using with below command for mapping:
  
  #ros2 launch cartographer_ros 2dLidar_mapping.py
  
  Noticed:
  modify below info in lua file:
  "-configuration_directory","/home/orion-jetsonnano/sunhuchang/ros2/cartographer_ros/cartographer_ros/configuration_files"
 
  

  
