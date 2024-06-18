(system-architecture)=
# System Architecture

```{note}
Please note that we are still actively working on achieving our goals and milestones. Therefore, this document does not fully reflect the current state of the module. It is only intended to briefly cover the entire system architecture, considering our vision and goals.
```

## 1. Introduction

### 1.1. Purpose
The purpose of this document is to outline the design of the system, an open-source, high-performance 2D/3D SLAM module built on the Robot Operating System (ROS) with OpenVDB as the map storage backend. It is designed for efficient map management, real-time capabilities, and modular plugin-based architecture to support multiple sensor types and easy customization.

### 1.2. Scope
This document covers the high-level system architecture, sequence of operations, and key design considerations for implementing the SLAM pipeline. It is intended for software developers and system architects involved in the development and maintenance of SLAM-VDB. Detailed implementation specifics are beyond the scope of this document; interested users can refer to the following links for more information.



### 1.3. Useful Links

## 2. Background and Context

### 2.1. Existing Works
SLAM systems are used in robotics to build a map of an unknown environment while simultaneously keeping track of the robot's location within that map. Existing open-source SLAM packages face challenges in terms of performance, scalability, and ease of customization.

```{todo}
Add literature review.
```

### 2.2. Problem Statement
Existing SLAM systems often struggle with high CPU/Memory usage, limited scalability for large maps, and difficulty in integrating multiple sensor types. There is a need for an open-source, high-performance, easily configurable SLAM system that can handle large-scale environments and it should just work out of the box.

### 2.3. Objectives
The objective of SLAM-VDB is to provide a robust and flexible SLAM solution that is:

- Sensor Agnostic
- Easily configurable and extensible via plugins
- Efficient map storage and real-time rendering capabilities
- Handle large-scale environments
- Support lifelong mapping
- High Performanace
- Ease of use with extensive documentation

## 3. Requirements

### 3.1. Functional Requirements
- Support for multiple sensor types (2D/3D LiDAR, cameras, IMUs)
- Configurable front-ends and back-ends via plugins
- Real-time map rendering capabilities
- Serialization and deserialization of maps and pose graphs
- Single configuration file for entire system setup

### 3.2. Non-Functional Requirements
- High performance with low CPU/Memory usage
- Scalability to handle large maps
- Modular and extensible design

### 3.3. User Requirements
- Easy setup and configuration
- Comprehensive documentation
- Ability to customize and extend functionalities via plugins

## 4. High-Level Design

### 4.1. System Architecture
SLAM-VDB is composed of several key components, each responsible for a specific aspect of the SLAM process. The architecture is modular and plugin-based, allowing for easy customization and extension.

High-Level Architecture Diagram
```{image} ../img/system_architecture/slamvdb-system-architecture.png
:align: center
```

### 4.2. Data Flow

### 4.3. Component Design

```{image} ../img/system_architecture/slamvdb-front-end-sequence-diagram.png
:align: center
```

## 5. Detailed Design

## 6. Configuration

## 7. Design Considerations

### 7.1 Performance

### 7.2 Extensibility

### 7.3 Real-Time Capabilities

### 7.4 Future Plans

## References