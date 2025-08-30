# ROS2 Temperature Monitor
This is a ROS2 system using Docker containers where a Python node publishes simulated temperature readings and a C++ node subscribes, processes, and displays temperature statistics.

This is my attempt at the first exercise by the robotics software community (RobotX).

Further details about the exercise are in the *Exercise_details.md* file.

## Tasks checklist
### Main Tasks:

- [x] Dockerfiles & Docker-compose setup
- [x] Custom temperature message 
- [x] Python Publisher Implementation
- [x] C++ Subscriber Implementation
  

### Advanced Features:
- [x] QoS settings for reliable communication
- [x] Implement graceful shutdown handling <small>*(implemented for publisher and subsciber containers only)*</small>
- [x] Add logging with different severity levels <small>*(INFO & WARNINGS only)*</small>
- [x] Launch file that starts both containers
### Challenges
- [x] Third container with a web-based visualization
- [ ] Data persistence using a database container
- [ ] Unit tests for both nodes
- [ ] Performance monitoring and metrics collection

## Usage
 
1. Clone this branch: ```git clone -b temp_monitor https://github.com/YouhanaBeshay/robotics-software-community.git```
2. Change directory: ```cd 1-temperature-monitor```
3. Compose up with build flag: ```docker compose up --build ```
4. ***for web-based visualization***: Open your browser at ```http://localhost:8080```
