## Usage Guide


Step 1: Install required Python packages (mentioned in the scripts) and ensure you are running Ubuntu version 20.04 specifically

Step 2: Connect to the robot over the network using ssh

Step 3: Launch the Start the robot using dinova.launch as explained in [REF] and launch the camera using:

source /opt/ros/noetic/setup.bash
source ~/bep_realsense_ws/devel/setup.bash
Then launch the camera with:
LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5.4 /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5.4 /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5.4 /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.5.4" roslaunch realsense2_camera rs_camera.launch

Step 4: Create a folder named lab_data_recorder at the location where you want to store the python and bash files

Step 5: Run the automated bash command: ~/lab_data_recorder/start_recorder.sh

Make sure that the right ROS topics are being recorded for your specific application. This can be changed in the python files itself.
