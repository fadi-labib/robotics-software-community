# ROS2 Docker Exercise: Cross-Language Temperature Monitor

## Exercise Overview
Design and implement a distributed ROS2 system using Docker containers where a Python node publishes simulated temperature readings and a C++ node subscribes, processes, and displays temperature statistics.

## Learning Objectives
By completing this exercise, students will:
1. Configure Docker containers for ROS2 applications
2. Set up inter-container networking using Docker Compose
3. Implement ROS2 publishers in Python (rclpy)
4. Implement ROS2 subscribers in C++ (rclcpp)
5. Handle ROS2 message serialization across languages
6. Debug distributed ROS2 systems in containerized environments

## Prerequisites
- Basic knowledge of ROS2 concepts (nodes, topics, messages)
- Familiarity with Python and C++
- Docker and Docker Compose installed
- Understanding of basic networking concepts

## Exercise Tasks

### Task 1: Environment Setup
1. Create a project structure with separate directories for Python and C++ nodes
2. Write Dockerfiles for both containers based on ROS2 Humble
3. Configure a docker-compose.yml file with proper networking

### Task 2: Message Definition
1. Create a custom message type for temperature data including:
   - Temperature value (float)
   - Timestamp
   - Sensor ID (string)
   - Unit (Celsius/Fahrenheit)

### Task 3: Python Publisher Implementation
1. Create a Python node that publishes temperature readings every second
2. Implement realistic temperature variations (sine wave + random noise)
3. Add command-line arguments for:
   - Publishing frequency
   - Temperature range
   - Sensor ID

### Task 4: C++ Subscriber Implementation
1. Create a C++ node that subscribes to temperature data
2. Calculate and display:
   - Moving average (last 10 readings)
   - Min/max temperatures
   - Temperature trend (rising/falling/stable)
3. Implement a warning system for temperatures outside normal range

### Task 5: Advanced Features (10 points)
1. Add QoS settings for reliable communication
2. Implement graceful shutdown handling
3. Add logging with different severity levels
4. Create a launch file that starts both containers

### Bonus Task: Challenges
1. Add a third container with a web-based visualization (ROS2 bridge to WebSocket)
2. Implement data persistence using a database container
3. Create unit tests for both nodes
4. Add performance monitoring and metrics collection

## Hints and Guidelines

### Docker Networking
- Use a custom bridge network in docker-compose
- Ensure containers can resolve each other by name
- ROS2 uses DDS which requires specific ports - consider using host networking or properly configured bridge

### ROS2 Discovery
- Set ROS_DOMAIN_ID consistently across containers
- Configure DDS discovery for Docker (may need to set RMW_IMPLEMENTATION)

### Message Synchronization
- Use proper QoS profiles for real-time data
- Handle message drops gracefully
- Consider time synchronization between containers

### Debugging Tips
- Use `ros2 topic list` and `ros2 topic echo` for debugging
- Check DDS discovery with `ros2 multicast receive/send`
- Monitor container logs with `docker-compose logs -f`

## Evaluation Criteria
- Code quality and organization
- Proper use of ROS2 concepts
- Docker configuration correctness
- Error handling and robustness
- Documentation and comments
- Advanced features implementation

## Resources used

- Folder structure: https://github.com/WATonomous/wato_monorepo/blob/main/docs/monorepo.md
