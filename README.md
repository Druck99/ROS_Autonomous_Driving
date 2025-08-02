# Autonomous Driving  
by Team 14: *Autonomous Driving*

## Description  
This is a project for the course **Introduction to ROS** in SS25 at TUM.  
The goal of the project is to enable a fully autonomous vehicle to drive through an urban environment on a predefined track, as fast as possible, while remaining on the road, avoiding collisions, and obeying traffic lights.

Unlike other solutions, our implementation **uses a semantic camera** to detect and classify objects like vehicles and traffic lights, and we additionally calculate the **relative distance** between our ego vehicle and detected objects to inform behavior decisions.

---

### ✅ Completed Missions  
All project requirements have been successfully fulfilled:

- ✅ Functional **perception pipeline** using semantic segmentation  
- ✅ Working **path planning** module  
- ✅ Working **trajectory planning** module  
- ✅ Accurate **vehicle detection and avoidance**  
- ✅ **Traffic light recognition** and compliant stopping/starting behavior  
- ✅ **Distance estimation** to both vehicles and traffic lights  
- ✅ Total completion time: ~400s  
- ✅ All modules integrated and verified in a simulated urban environment

## Team Contributions

| Assignments                           | Contributors     | Reuse Package            |
|---------------------------------------|------------------|--------------------------|
| Map construction                      | Enze Wang           | Yes, `octomap` |
| Point cloud generation                | Enze Wang           | Yes, `depth_image_proc`, `pcl_ros`           |
| Perception - Traffic light recognition| Enze Wang          | No          |
| Perception - Distance to traffic lights | Enze Wang        | No                       |
| Perception – Distance to vehicles        | Enze Wang    | Yes, `cv_bridge`, `message_filters`       |
| State machine                         | Yicun Song          | No                       |
| Global planner                        | Jiaxiang Yang           | Yes, `move_base`         |
| Local planner                         | Jiaxiang Yang    | Yes, `TebLocalPlanner`   |
| Controller                            | Qianke Ye    | No                       |


---

## Installation Guide  

You can run the project locally. **GPU acceleration is not required** as we do not use deep learning models such as YOLO.

### 1. Local Installation  
Assume you already have `ros-noetic-desktop-full` installed.

1. Install dependencies:
   ```bash
   sudo apt-get update && sudo apt-get install -y \
   python3-rosdep \
   python3-rosinstall \
   python3-rosinstall-generator \
   python3-wstool \
   build-essential \
   python3-catkin-tools \
   ros-noetic-rviz \
   ros-noetic-octomap-server \
   ros-noetic-navigation \
   ros-noetic-octomap-rviz-plugins \
   ros-noetic-teb-local-planner
2. Build the project using `catkin build` under your `catkin_ws` directory:

   ```bash
   cd ~/catkin_ws
   catkin build
   ```

3. Source the workspace:

   ```bash
   source devel/setup.bash
   ```

4. Launch the simulator and project:

   ```bash
   roslaunch simulation simulation_demo.launch
   ```

> 📌 ROS graph after successful build:
> ![rosgraph](your_rosgraph_image_path_here)


## Support

For further questions, feel free to contact us at: **jiaxiang.yang@tum.de**

---

## Roadmap

* Further tuning of local planner parameters for better trajectory smoothness
* Adding more object types to enhance decision-making in dynamic environments

---

## Authors and Acknowledgment

Enze Wang, Jiaxiang Yang, Yicun Song, Qianke Ye

---

## License

MIT License

---

## Project Status

All modules are functioning correctly in the simulation. Additional improvements and real-world validation may be considered in the future.
