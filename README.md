# AntoSim

- - -

## 🧑‍💻 System Requirements

- Ubuntu 22.04  
- ROS 2 Humble  
- Gazebo 11  
- Python packages:
  - `flask`, `paho-mqtt`, `opencv-python`, `cv_bridge`  
- MQTT broker:
  - `mosquitto`  
- Flutter (for mobile app)
- Android Studio (for mobile development)

- - -

## ⚙️ Installing Dependencies

Install ROS 2 and Gazebo:

~~~bash
sudo apt update
sudo apt install ros-humble-desktop gazebo libgazebo11-dev
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher ros-humble-robot-state-publisher
~~~

Install Python packages:

~~~bash
pip install flask paho-mqtt opencv-python
~~~

Install and enable Mosquitto (MQTT broker):

~~~bash
sudo apt install mosquitto
sudo systemctl enable --now mosquitto
~~~

- - -

## 🛠️ Building the Package

~~~bash
cd ~/Documents/Real_antosim/AntoSim/antobot_ant_description
colcon build --packages-select antobot_ant_description
source install/setup.bash
~~~

- - -

## 🚀 Running the Full System

Launch everything from a single command:

~~~bash
ros2 launch antobot_ant_description full_system.launch.py \
  world:=$HOME/.gazebo/worlds/agriculture.world
~~~

This will:
- Launch Gazebo with the selected world  
- Spawn the robot  
- Start the camera MJPEG stream  
- Launch the joystick MQTT controller  

- - -

## 🎮 Joystick Control & MJPEG Camera via Mobile App

You can control the robot and view the camera stream using the mobile app located at:

AntoSim/
├── antobot_ant_description/ # ROS 2 package with robot description, camera + joystick nodes
├── antobot_sim_description/ # Optional: related simulation assets
└── App/ # Flutter mobile app (UI + MQTT + video stream)
└── lib/
└── main.dart


- - -

## 🧑‍💻 System Requirements

- Ubuntu 22.04  
- ROS 2 Humble  
- Gazebo 11  
- Python packages:
  - `flask`, `paho-mqtt`, `opencv-python`, `cv_bridge`  
- MQTT broker:
  - `mosquitto`  
- Flutter (for mobile app)
- Android Studio (for mobile development)

- - -

## ⚙️ Installing Dependencies

Install ROS 2 and Gazebo:

~~~bash
sudo apt update
sudo apt install ros-humble-desktop gazebo libgazebo11-dev
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher ros-humble-robot-state-publisher
~~~

Install Python packages:

~~~bash
pip install flask paho-mqtt opencv-python
~~~

Install and enable Mosquitto (MQTT broker):

~~~bash
sudo apt install mosquitto
sudo systemctl enable --now mosquitto
~~~

- - -

## 🛠️ Building the Package

~~~bash
cd ~/Documents/Real_antosim/AntoSim/antobot_ant_description
colcon build --packages-select antobot_ant_description
source install/setup.bash
~~~

- - -

## 🚀 Running the Full System

Launch everything from a single command:

~~~bash
ros2 launch antobot_ant_description full_system.launch.py \
  world:=$HOME/.gazebo/worlds/agriculture.world
~~~

This will:
- Launch Gazebo with the selected world  
- Spawn the robot  
- Start the camera MJPEG stream  
- Launch the joystick MQTT controller  

- - -

## 🎮 Joystick Control & MJPEG Camera via Mobile App

You can control the robot and view the camera stream using the mobile app located at:

AntoSim/App/lib/main.dart


To run the app:

~~~bash
cd AntoSim/App
flutter pub get
flutter run
~~~

- - -

## 📱 Installing Flutter and Android Studio

Install Flutter:

~~~bash
sudo snap install flutter --classic
flutter doctor
~~~

Install Android Studio (via Snap or manually):

~~~bash
sudo snap install android-studio --classic
~~~

Then:

1. Open Android Studio  
2. Install Flutter and Dart plugins  
3. Set up an Android emulator or connect a physical device  
4. Open the `AntoSim/App` project and press ▶️ **Run**

If joystick commands aren’t working in the app, just **restart it**.

- - -

## 🔧 Testing MQTT Commands Manually

Subscribe:

~~~bash
mosquitto_sub -h localhost -t antobot/joystick
~~~

Publish test command:

~~~bash
mosquitto_pub -h localhost -t antobot/joystick -m '{"x": 1.0, "y": -0.5}'
~~~

- - -

## 📷 Viewing MJPEG Stream from Robot

Open in your browser:

http://localhost:5000/video

Or, select the camera icon on the mobile app wehn connected to the same network as the device


- Moves forward or stays still → front camera  
- Moves backward → rear camera  

- - -

## 🧪 Debugging Tips

- No robot movement? Check `/cmd_vel` topic and MQTT  
- No image? Make sure Gazebo + robot is publishing `/front_camera/image_raw`  
- MQTT not connecting? Restart the mobile app first then Mosquitto if issues persist:
  ~~~bash
  sudo systemctl restart mosquitto
  ~~~




