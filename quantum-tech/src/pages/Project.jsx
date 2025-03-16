import React from "react";
import { Link } from "react-router-dom";

const Project = () => {
  return (
    <div className="project-page">
      <section className="page-hero">
        <div className="page-hero-content">
          <div className="page-badge">Featured Project</div>
          <h1 className="page-title">Quantum-Based Autonomous Navigation System</h1>
          <p className="page-subtitle">
            An integrated system for autonomous robot navigation combining ROS2, ML, 
            quantum computing, and real-time tracking
          </p>
        </div>
        <div className="page-hero-image">
          <img 
            src="https://scitechdaily.com/images/Autonomous-Self-Driving-Car-Technology.jpg" 
            alt="Autonomous Navigation Robot" 
          />
          <div className="page-hero-overlay"></div>
        </div>
      </section>

      <section className="page-section">
        <div className="container">
          <div className="project-overview">
            <h2 className="section-title">Project Overview</h2>
            <p className="section-text">
              This project aims to create an autonomous navigation system for a robot that can move from Point A 
              to Point B safely while adapting to real-world environments. The system integrates localization, 
              real-time obstacle detection, shortest path computation, and live communication, making use of 
              quantum technology.
            </p>
          </div>
        </div>
      </section>

      <section className="page-section bg-light">
        <div className="container">
          <h2 className="section-title">Key Components and Methodologies</h2>

          <div className="component-section">
            <h3 className="component-title">1. Localization Using ROS2</h3>
            <p className="section-text">
              To achieve precise navigation, ROS2 is used for Simultaneous Localization and Mapping (SLAM). This process involves creating a map of the environment while simultaneously keeping track of the robot's location within it. Sensors such as LIDAR and cameras are utilized to gather data, which is then processed to build a detailed map. SLAM is crucial for autonomous navigation as it allows the robot to understand and adapt to its surroundings in real-time.
            </p>
            
            <div className="code-section">
              <h4>Steps to Create a Map</h4>
              
              <div className="code-block">
                <h5>Launching TurtleBot3:</h5>
                <pre><code>ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py</code></pre>
              </div>
              
              <div className="code-block">
                <h5>Opening RViz for Mapping:</h5>
                <pre><code>ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True</code></pre>
              </div>
              
              <div className="code-block">
                <h5>Moving the Robot for Mapping:</h5>
                <p><strong>Manually:</strong></p>
                <pre><code>ros2 run turtlebot3_teleop teleop_keyboard</code></pre>
                <p><strong>Automatically:</strong></p>
                <pre><code>ros2 run turtlebot3_gazebo turtlebot3_drive</code></pre>
              </div>
              
              <div className="code-block">
                <h5>Saving the Generated Map:</h5>
                <pre><code>ros2 run nav2_map_server map_saver_cli -f maps/my_map</code></pre>
              </div>
            </div>
            
            <div className="map-legend">
              <h4>Understanding the Map Representation:</h4>
              <ul>
                <li><span className="map-color black"></span> <strong>Black</strong> - Obstacles</li>
                <li><span className="map-color white"></span> <strong>White</strong> - Free Space</li>
                <li><span className="map-color brown"></span> <strong>Brown</strong> - Unknown Space</li>
              </ul>
            </div>
          </div>

          <div className="component-section">
            <h3 className="component-title">2. Obstacle and Environment Detection Using ML YOLO Models</h3>
            <p className="section-text">
              The system employs YOLO (You Only Look Once) machine learning models along with a camera to detect various elements in the environment. YOLO's architecture allows for fast and accurate object detection, making it ideal for real-time applications. The models are trained on a diverse dataset to recognize obstacles, potholes, vehicles, and dynamic elements, enhancing the robot's decision-making capabilities.
            </p>
            <ul className="feature-list">
              <li>
                <span className="feature-icon">🚧</span>
                <div>
                  <h3>Obstacles</h3>
                  <p>Detection of static obstacles in the environment</p>
                </div>
              </li>
              <li>
                <span className="feature-icon">🕳️</span>
                <div>
                  <h3>Potholes</h3>
                  <p>Identification of terrain hazards like potholes</p>
                </div>
              </li>
              <li>
                <span className="feature-icon">🚗</span>
                <div>
                  <h3>Vehicles</h3>
                  <p>Recognition of moving and stationary vehicles</p>
                </div>
              </li>
              <li>
                <span className="feature-icon">🔄</span>
                <div>
                  <h3>Dynamic Elements</h3>
                  <p>Detection of other moving objects in the Indian environment</p>
                </div>
              </li>
            </ul>
            <p className="section-text">
              These detections enhance the robot's decision-making for safe navigation.
            </p>
          </div>

          <div className="component-section">
            <h3 className="component-title">3. Finding the Shortest Path Using Dijkstra's Algorithm</h3>
            <p className="section-text">
              Once the environment is mapped, the optimal path is calculated using Dijkstra's Algorithm. This algorithm is known for its efficiency in finding the shortest path between nodes in a graph, which is essential for navigating complex environments. By integrating quantum computing, the algorithm's performance is further enhanced, allowing for faster computation and real-time path adjustments.
            </p>
            <div className="algorithm-visualization">
              <img 
                src="https://ds055uzetaobb.cloudfront.net/brioche/uploads/lessons/dijkstra-EQ50NN.gif" 
                alt="Path Finding Algorithm Visualization" 
              />
            </div>
          </div>

          <div className="component-section">
            <h3 className="component-title">4. Real-Time Live Location Tracking Using Web API</h3>
            <p className="section-text">
              To ensure accurate tracking, a web API is integrated to provide live location updates. The API is built using modern web technologies, ensuring secure and efficient data transmission. It allows for external monitoring and control, providing stakeholders with real-time insights into the robot's location and status.
            </p>
            <ul>
              <li>This enables external monitoring and enhances the adaptability of the robot's movement.</li>
              <li>The location data is constantly updated to maintain accuracy in a dynamic environment.</li>
            </ul>
          </div>

          <div className="component-section">
            <h3 className="component-title">5. Continuous Communication for Remote Monitoring</h3>
            <p className="section-text">
              The system is designed to stay in continuous communication using advanced protocols that ensure reliable data transmission. This feature is critical for remote monitoring and control, allowing for instant updates on navigation status and external intervention if necessary.
            </p>
            <ul>
              <li>Real-time <strong>data transmission</strong> for monitoring.</li>
              <li>Instant updates on navigation status.</li>
              <li>External control capabilities in case of emergencies.</li>
            </ul>
          </div>

          <div className="component-section">
            <h3 className="component-title">6. Quantum Technology Integration</h3>
            <p className="section-text">
              This project leverages quantum technology to enhance computational efficiency in shortest path algorithms and optimize real-time decision-making processes. Quantum computing provides a significant speedup in processing complex calculations, making it a valuable asset in autonomous navigation systems.
            </p>
            <ul>
              <li><strong>Computational efficiency</strong> in shortest path algorithms.</li>
              <li><strong>Optimization</strong> of real-time decision-making processes.</li>
              <li><strong>Improved security</strong> in data transmission and communication.</li>
            </ul>
          </div>

          <div className="component-section">
            <h3 className="component-title">7. Robot Navigation with Predefined Pathing</h3>
            
            <div className="code-section">
              <div className="code-block">
                <h5>Importing the Map in RViz:</h5>
                <pre><code>ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/my_map.yaml</code></pre>
              </div>
            </div>
            
            <p className="section-text">
              <strong>Setting Destination Points:</strong>
            </p>
            <ul>
              <li>The robot autonomously navigates to the set destination.</li>
              <li>Multiple destinations can be set for planned pathing.</li>
            </ul>
            <p className="section-text">
              The robot is capable of navigating to predefined destinations autonomously. This feature allows for efficient pathing in dynamic environments, with the ability to adjust routes in real-time based on new data and obstacles encountered.
            </p>
          </div>
        </div>
      </section>

      <section className="page-section">
        <div className="container">
          <h2 className="section-title">Conclusion</h2>
          <p className="section-text">
            This project successfully integrates ROS2 localization, ML-based obstacle detection, 
            Dijkstra's Algorithm for shortest pathfinding, real-time web API tracking, and quantum 
            technology for optimized computation. With continuous communication and live updates, 
            this system is well-suited for autonomous navigation in dynamic Indian environments.
          </p>
        </div>
      </section>

      <section className="cta-section">
        <div className="container">
          <div className="cta-content text-center">
            <h2>Interested in Learning More?</h2>
            <p>Check out our other cutting-edge technology domains</p>
            <div className="cta-buttons">
              <Link to="/quantum" className="btn btn-primary">
                Quantum Physics
              </Link>
              <Link to="/yolo" className="btn btn-secondary">
                YOLO Detection
              </Link>
              <Link to="/routing" className="btn btn-tertiary">
                Routing Systems
              </Link>
              <Link to="/" className="btn btn-quaternary">
                Back to Home
              </Link>
            </div>
          </div>
        </div>
      </section>
    </div>
  );
};

export default Project; 