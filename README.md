## Installation and execution

1. Clone this package within the `src` folder of your workspace using ```git clone git@github.com:guptaPrabhav/usb_cam_custom.git```.
2. source your ros installation - ```source /opt/ros/foxy/setup.bash```.
3. Navigate to the workspace and build the package using ```colcon build --packages-select usb_cam_custom```.
4. within the same folder where you built the package, source your package installation running ```source install/setup.bash```.
5. execute the following ros launch command ```ros2 launch usb_cam_custom change_image_mode.launch.py```. This should launch the action server.
6. In a new terminal, source ros installtion as done previously. Then use the following two commands to change the mode of the output image - 
    - `ros2 action send_goal /change_image_mode usb_cam_custom/action/ChangeImageMode grayscale:\ true\` to change the image view from RGB to grayscale.
    - `ros2 action send_goal /change_image_mode usb_cam_custom/action/ChangeImageMode grayscale:\ false\` to change the image view from grayscale to RGB.

The output image will be published on the topic `/output_image`. View the topic via RVIZ to verify

Conda issue: https://github.com/ros2/examples/issues/303