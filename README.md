Current setup: simple image subscriber using OpenCV and Cpp

1. Open 2 terminal windows
2. change directory using `cd ~/mowito_ws/` in both the terminals
3. execute `source /opt/ros/foxy/setup.bash` and `source install/setup.bash` in both the terminals
4. In terminal A, execute: `ros2 run usb_cam usb_cam_node_exe`
5. In termianl B, execute: `ros2 run usb_cam_custom image_proc`
