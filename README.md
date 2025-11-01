# hand_palm_ur10
## Teleoperating UR10 with hand palm position from 2D Webcam - ROS2 Jazzy
### Hand dectection using MediaPipe and simulate UR10 on URSim

Initialize virtual environment for installing MediaPipe:
```
cd ~/colcon_ws
python3 -m venv ros_venv
source ~/colcon_ws/ros_venv/bin/activate
pip install mediapipe opencv-python
```
Init Palm Coordinate Node:
```
ros2 run hand_palm_ur10 palm_detector
```
Init Camera to Image Node (topic `\image`):
```
ros2 run image_tools cam2image
```
Init Palm to URScript Node:
```
ros2 run hand_palm_ur palm_controller
```
Init URSim (take the IP of the simulated robot from here):
```
sudo docker run --rm -it -p 5900:5900 -p 30001-30004:30001-30004 universalrobots/ursim_e-series:5.11
```
Init `ur_robot_driver`:
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=YOUR_ROBOT_IP
```
