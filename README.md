# hand_palm_ur10
## Teleoperating UR10 with hand palm position from 2D Webcam - ROS2 Jazzy
### Hand dectection using MediaPipe and simulate UR10 on URSim
### Idea:
- Use Mediapipe to detect 21 landmarks of the hand
- 2 methods of getting palm coordinates:
  1. Extract the center of Wrist and Middle Finger coordinates (only x and y)
     ![](https://github.com/trungtran22/hand_palm_ur10/blob/main/docs/MediaPipe%20Hands_screenshot_01.11.2025.png)
  2. Create a bounding box and get the center of the bounding box (x and y); area of bounding box represent z coordinate
     ![](https://github.com/trungtran22/hand_palm_ur10/blob/main/docs/Palm%20Detector_screenshot_01.11.2025.png)
- Convert to `String_msgs` and publish to URSim via topic `/urscript_interface/script_command`
  - speedl([vx,vy,vz, rx,ry,rz], acceleration, time)
### Installation and Initialize
Initialize virtual environment for installing MediaPipe:
```
cd ~/colcon_ws
python3 -m venv ros_venv
source ~/colcon_ws/ros_venv/bin/activate
pip install mediapipe opencv-python
```
Create Package for the project:
1. Creating package with dependencies
```
ros2 pkg create --build-type ament_python hand_palm_ur10 --dependencies rclpy sensor_msgs geometry_msgs cv_bridge
```
2. Download 2 codes in the hand_palm_ur10 folder; save it inside ~/colcon_ws/src/hand_palm_ur10/hand_palm_ur10 
3. Config setup.py: edit entry_points with below lines; you can copy from setup.py file in the repository
```
entry_points={
        'console_scripts': [
            'palm_detector = hand_detector.palm_hand_publish:main',
            'palm_controller = hand_detector.palm_to_script:main', 
        ],
```
4. Build the package
```
cd ~/colcon_ws
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select hand_palm_ur10
```
5. Because we have to work in virtual environment for the mediapipe, we have to adjust python3 directory of the package inside the **install** folder; copy the line below, adjust `user_please_adjust_to_yours` to your computer user
```
gedit ~/colcon_ws/install/hand_palm_ur10/lib/hand_palm_ur10/palm_detector.py
### Please copy this line below and replace the current python3 directory of the file so that our project can work
#!/home/user_please_adjust_to_yours/colcon_ws/ros_venv/bin/python3
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
