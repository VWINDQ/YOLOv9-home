# YOLOv9-@Home Education 
YOLOv9's Team Robocup Malaysia open 2024 @Home Education

This project uses Turtlebot2, ROS(Robot Operating System),Kinect camera and YDlidar to do tasks at home
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

#### 2. Create catkin Workspace

