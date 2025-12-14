# Quickstart Guide: Setting up the Development Environment

This guide provides the steps to set up the development environment for creating and validating the content for the book "Physical AI & Humanoid Robotics."

## System Requirements

- **OS**: Ubuntu 22.04 LTS
- **Hardware**:
    - High-Performance Workstation with NVIDIA RTX 4070 Ti or higher.
    - NVIDIA Jetson Orin Nano Developer Kit.

## Software Installation

1.  **Install ROS 2 Humble Hawksbill**: Follow the official ROS 2 installation guide.
2.  **Install Gazebo**: Install the latest version of Gazebo that is compatible with ROS 2 Humble.
3.  **Install Unity**: Install Unity Hub and the latest LTS version of the Unity Editor.
4.  **Install NVIDIA Isaac Sim**: Follow the installation instructions for NVIDIA Isaac Sim.
5.  **Install Docusaurus Dependencies**: Install Node.js and npm, then run `npm install` in the project root to install the necessary Docusaurus dependencies.

## Validation

After installation, run the following commands to ensure that all components are working correctly:

- `ros2 doctor`
- `gazebo --version`
- `unity -version`
- (Isaac Sim has a GUI-based validation)
- `npm run start` (in the Docusaurus project directory)
