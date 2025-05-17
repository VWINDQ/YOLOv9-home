# YOLOv9-@Home Education 
YOLOv9's Team | Robocup Malaysia open 2024 @Home Education

This project is developed by YOLOv9's Team Robocup Malaysia for the 2024 @Home Education competition, aiming to use Turtlebot2 robots equipped with ROS, Kinect camera and YDLidar to perform various tasks within the home.
## Feature
- Detects and Follow people
- Announces detected person's gender and shirt color
- Object Detection
- Supports Thai and English voice output using gTTS
- Face Recognition
## Member
- Mr.Phattaraphon Suttiphan
- Mr.Chayakorn Winothai
- Mr.Pichanat Sittipunyota
- Ms.Natwalee Narhkthong
## Technologies Used
- **Robot Operation System (ROS):noetic**
- **Python**
- **Turtlebot2**
- **Kinect camera**
- **YDlidar X2**
- **OpenCV, YOLOv5 (Image Processing)**
- **gTTS (Google Text-To-Speech)**
- **RViz,Gazebo (For Simulation)**
- **Robotic Arm (MG996R Servo)**
- **Arduino IDE (For Robotic arm)**
  
[Technical Poster](Rules\Poster-YOLOv9.pdf)


[Rule](Rules\Rule2024RoboCup@HomeEducation.pdf)

---

## 🚀 Installation and Usage Guide

### ✅ Prerequisites
- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3
- Arduino IDE (for controlling the robotic arm)
- YDLidar SDK (compatible with YDLidar X2)
- Serial port access for devices connected via USB

---

### 🛠 System Installation Steps

#### 1. Install ROS Noetic
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```
#### 2. Create catkin Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
#### 3. Clone the Project into Your Workspace
```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/smart-home-robot.git
cd ~/catkin_ws
catkin_make
```
#### 4. Add Workspace to .bashrc
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### 5. Connect Hardware
* TurtleBot2 via USB or WiFi (with ROS driver)
* YDLidar X2 via /dev/ttyUSB0 (or as assigned)
* Arduino UNO (controlling 3 MG996R servos) via /dev/ttyUSB1

### ▶️ Running the System

#### 1. Start ROS Core
```bash
roscore
```
#### 2. Launch YDLidar
```bash
roslaunch ydlidar_ros X2.launch
```
#### 3. Launch TurtleBot2 Navigation with Map
```bash
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/path/to/your/map.yaml
```
#### 4. Run the Robotic Arm Control Script
```bash
rosrun smart_home_robot move_arm.py
```
#### 5. Launch the Full Robot System
```bash
roslaunch smart_home_robot bringup.launch
```
### 🧪 Testing the System
* Verify TurtleBot2 moves to designated goals.
* Check YDLidar sensor correctly detects obstacles and builds the map.
* Test the robotic arm to ensure it picks and moves objects properly.
* Run end-to-end tasks (e.g., robot picks and delivers an object).

📌 Note: If USB devices are not recognized, check port permissions and ensure that drivers for YDLidar and Arduino are properly installed.
