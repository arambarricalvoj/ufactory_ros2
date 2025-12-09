ros2 launch camera_robot launch.py
ros2 run ros_gz_bridge parameter_bridge /world/default/model/my_robot/link/camera_link/sensor/camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image
ros2 topic echo /world/default/model/my_robot/link/camera_link/sensor/camera_sensor/image

https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors?tab=readme-ov-file#rgbd-camera
