Team 5 - 4020 Project (MME)

Detects Thumbs Up, Thumbs Down, and Unknown gestures from a live camera using MediaPipe and a trainable MLP classifier. Label gestures in real time to build a personalised model that improves as you collect more data.


SETUP
Install dependencies:
  pip install opencv-python mediapipe scikit-learn numpy
Run the script:
  python3 Thumbs_Reinforcement.py


LABELLING
While the camera is running, show a gesture and press:
  U = Thumbs Up
  D = Thumbs Down
  X = Unknown
  Q = Quit

The app starts with a simple rule-based classifier. Once you hit 30 labelled samples it trains the ML model automatically and switches to it. The model retrains every 10 new labels after that.


HOW MANY SAMPLES
Aim for 150-200 total, roughly split evenly across the three gestures. More variety (different distances, angles, lighting) matters more than raw count.


SAVING
Data is saved every time you press a label key, so nothing is lost if the app crashes. The model is saved after every training run. Both reload automatically on the next launch.


CAMERA CONFIGURATION
At the top of the script you can change the CAMERA_INDEX variable using 0 for built-in, 1+ for external cameras


MOCK PICK MODE (PC ONLY)
Use this to test ROS request -> plan/execution flow without connecting the robot.

1) Start backend (terminal A):
  python3 mock_pick_backend.py

2) Send one test request (terminal B):
  python3 mock_pick_client.py --color red --x 0.20 --y 0.00 --z 0.03 --yaw 0.0 --place red_bin

3) Optional failure simulation:
  python3 mock_pick_backend.py --ros-args -p fail_rate:=0.25
This randomly fails 25% of requests to test recovery handling.


MOCK CAMERA -> PICK PIPELINE (OFFICIAL AI-KIT IMAGE)
This replays an Elephant Robotics AI-kit sample picture as `/camera/image`,
detects colored blocks, generates a pick motion plan, and runs mock execution.

1) Source ROS:
  source /opt/ros/galactic/setup.bash

2) Run the full pipeline:
  ros2 launch ./mock_camera_pick_pipeline.launch.py show_window:=true target_color:=red

3) Headless mode:
  ros2 launch ./mock_camera_pick_pipeline.launch.py show_window:=false

4) Use your own picture:
  ros2 launch ./mock_camera_pick_pipeline.launch.py image_path:=/absolute/path/to/image.png target_color:=red


MOCK CAMERA + PICK + RVIZ MOTION
This version also animates the myCobot 280 Pi model in RViz from the planned pick waypoints.

1) Source ROS + myCobot workspace:
  source /opt/ros/galactic/setup.bash
  source /home/joenotexotic/ros2_ws/install/setup.bash

2) If using VS Code Snap terminal, clear Snap vars first (RViz crash workaround):
  for v in $(env | cut -d= -f1 | grep -E '^(SNAP|GTK_|GDK_|GIO_|LOCPATH)'); do unset "$v"; done

3) Launch full pipeline with RViz:
  ros2 launch ./mock_camera_pick_with_rviz.launch.py target_color:=red show_window:=false

4) Keep suction cup top-down (default) or tune orientation:
  ros2 launch ./mock_camera_pick_with_rviz.launch.py target_color:=red show_window:=false tool_pitch_rad:=-1.5708
If the cup is still tilted, try `tool_pitch_rad:=-1.35` or `tool_pitch_rad:=-1.75`.

5) Set designated bin positions (example):
  ros2 launch ./mock_camera_pick_with_rviz.launch.py target_color:=red show_window:=false red_bin_x:=0.18 red_bin_y:=-0.14


REAL ROBOT EXECUTION (myCobot VIA SERIAL)
Use this to run the same planning pipeline with real movement on the robot.

1) Install robot SDK:
  pip3 install pymycobot

2) Check serial port permissions:
  sudo usermod -a -G dialout $USER
  # log out/in, then verify your robot port (example: /dev/ttyUSB0)

3) Run real pipeline:
  ros2 launch ./real_camera_pick_pipeline.launch.py port:=/dev/ttyUSB0 baud:=115200 target_color:=red

4) Tool control mode:
  # suction via basic output (default)
  ros2 launch ./real_camera_pick_pipeline.launch.py tool_action_mode:=basic_output

  # skip pump/gripper commands
  ros2 launch ./real_camera_pick_pipeline.launch.py tool_action_mode:=none

Note: `real_pick_backend.py` executes motion_plan poses from `/pick/request` and publishes statuses on `/pick/status`.


WINDOWS -> RASPBERRY PI (FULL SETUP + RUN)
Use this when your code is on Windows and you want to run the real robot from the Pi.

1) Copy project folder from Windows to Pi:
  scp -r C:\Users\highf\Downloads\4020_Project\4020_Project\group-project-group-5 pi@<PI_IP>:~/4020_Project/

2) SSH into Pi:
  ssh pi@<PI_IP>

3) Install dependencies on Pi:
  sudo apt update
  sudo apt install -y python3-pip python3-opencv ros-galactic-cv-bridge ros-galactic-v4l2-camera
  pip3 install pymycobot

4) Set serial permissions once:
  sudo usermod -a -G dialout $USER
  # log out and log back in after this

5) Start from project folder and source ROS:
  cd ~/4020_Project/group-project-group-5
  source /opt/ros/galactic/setup.bash
  source ~/ros2_ws/install/setup.bash
  chmod +x *.py

6) Confirm camera and robot device names:
  ls /dev/video*
  ls /dev/ttyUSB* /dev/ttyACM*

7) Run LIVE camera + real robot (if your camera is /dev/video0):
  # Terminal 1 (camera -> /camera/image)
  ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0 -r image_raw:=/camera/image

  # Terminal 2 (block detection)
  python3 block_detector.py --ros-args -p show_window:=true -p min_area:=400 -p publish_period_sec:=1.0

  # Terminal 3 (pick + path planner node)
  python3 pickup_path_planner_node.py --ros-args -p target_color:=red

  # Terminal 4 (REAL robot backend, safe first run without tool action)
  python3 real_pick_backend.py --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200 -p speed:=20 -p tool_action_mode:=none

8) After motion is verified, enable pump control:
  python3 real_pick_backend.py --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200 -p speed:=20 -p tool_action_mode:=basic_output
