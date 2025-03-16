

# **Quantum-Based Autonomous Navigation System 🚀**
**A research-driven project integrating ROS2, YOLO-based obstacle detection, Dijkstra’s algorithm, live tracking, and quantum computing to achieve safe and intelligent autonomous navigation.**

## **🌍 Project Overview**
This project focuses on **developing an autonomous robotic system** capable of navigating through complex environments using **Simultaneous Localization and Mapping (SLAM)**, **real-time object detection**, and **optimal pathfinding**. It combines traditional AI-driven techniques with **quantum computing enhancements** to achieve improved efficiency.

## **🎯 Key Objectives**
✔ **Localization & Mapping:** Create and update dynamic maps using **ROS2 SLAM**.  
✔ **Obstacle Detection:** Use **YOLO-based machine learning** to identify obstacles, potholes, and vehicles.  
✔ **Shortest Path Calculation:** Implement **Dijkstra’s algorithm** for optimal path planning.  
✔ **Live Tracking:** Use **Web API** to fetch real-time robot location.  
✔ **Quantum Optimization:** Enhance pathfinding & decision-making using **quantum computing techniques**.  
✔ **Continuous Communication:** Maintain real-time updates for efficient remote monitoring.  

---

## **🛠️ Technology Stack**
| **Category**       | **Tools/Frameworks**  |
|-------------------|--------------------|
| **Localization & Mapping** | ROS2, SLAM, Cartographer |
| **Obstacle Detection** | YOLOv5, OpenCV, TensorFlow |
| **Pathfinding** | Dijkstra’s Algorithm |
| **Live Tracking** | Web API, Flask/Django, MQTT |
| **Communication** | ROS2 Navigation Stack, MQTT, WebSockets |
| **Quantum Optimization** | D-Wave Quantum Annealing, Qiskit |
| **Simulation** | TurtleBot3, Gazebo |

---

## **🚀 Implementation Details**
### **1️⃣ Localization & Mapping (SLAM with ROS2)**
- Use **ROS2 Cartographer** to generate a real-time map of the environment.
- Manually or automatically move the robot to collect **LiDAR** data.
- Save the generated map and **import it into RViz** for visualization.

**Commands:**
```sh
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
ros2 run turtlebot3_teleop teleop_keyboard  # Manual movement
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```

---

### **2️⃣ Object Detection & Hazard Avoidance (YOLOv5)**


### **3️⃣ Shortest Pathfinding (Dijkstra’s Algorithm)**
- Computes **fastest and safest** routes for navigation.
- Uses graph representation of the environment.

### **4️⃣ Real-Time Location Tracking (Web API)**
- Fetches **robot’s live location** using GPS or simulation coordinates.
- Displays real-time position on **a web dashboard**.


### **5️⃣ Quantum Optimization for Navigation**
- Uses **Quantum Annealing** to solve optimization problems.
- Applied to **multi-destination pathfinding** for enhanced efficiency.


## **🖥️ Setup & Installation**
### **Prerequisites**
- Ubuntu 20.04  
- ROS2 Foxy  
- Python 3.8+  
- Gazebo & TurtleBot3  
- OpenCV, YOLOv5, Flask/Django  
- D-Wave Ocean SDK (for quantum computing)  

### **Installation Steps**
```sh
# Install ROS2 Foxy
sudo apt update && sudo apt install ros-foxy-desktop-full

# Clone the repo
git clone https://github.com/yourusername/quantum-nav.git
cd quantum-nav

# Install dependencies
pip install -r requirements.txt

# Launch the simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## **📌 Features**
✔ **Dynamic SLAM-based Mapping**  
✔ **Real-time Obstacle Avoidance**  
✔ **Efficient Pathfinding with Dijkstra’s Algorithm**  
✔ **Live Location Tracking via Web API**  
✔ **Quantum-Assisted Route Optimization**  
✔ **Multi-Destination Navigation**  

 


