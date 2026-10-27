# ACC_2027_Competition
# 2027 QCar Project

## Overview

The 2027 QCar project focuses on improving the reliability, responsiveness, and overall performance of the autonomous driving system used on the Quanser QCar platform.

Development for this year will primarily focus on four areas:

- V2X communication
- Lane following
- Object detection
- System throughput and process optimization

The goal is to improve how the QCar perceives its environment, communicates with infrastructure and other vehicles, follows planned routes and lanes, and processes information in real time.

---

## Primary Development Areas

### V2X Communication

V2X, or Vehicle-to-Everything communication, allows the QCar to exchange information with surrounding infrastructure, vehicles, and other connected systems.

Development will focus on improving:

- Communication reliability
- Message timing and synchronization
- Traffic-light and infrastructure communication
- Vehicle-to-vehicle information exchange
- Integration of V2X data into vehicle decision-making
- Handling of delayed, missing, or inconsistent messages

The objective is for V2X information to become a dependable part of the QCar's autonomous decision-making pipeline.

---

### Lane Following

Lane following is responsible for keeping the vehicle accurately positioned along the desired driving path.

Development will focus on:

- Improving lane-centering accuracy
- Reducing steering oscillation
- Improving curve handling
- Improving recovery from lane deviation
- Producing smoother steering commands
- Improving the transition between perception, path generation, and steering control

The goal is to create more stable and repeatable driving behavior across different portions of the track.

---

### Object Detection

The QCar uses its perception system to identify objects and driving-related features in the environment.

Development will focus on improving:

- Detection accuracy
- Detection consistency
- Confidence threshold selection
- False-positive reduction
- Missed-detection reduction
- Real-time processing performance
- Vehicle responses to detected objects
- Predictive perception and short-horizon state estimation to support lane following, PID-based throttle control, steering control, and higher-level driving decisions.

Detection performance will be evaluated not only by whether an object is identified, but also by whether the QCar responds to that detection correctly and safely.

---

### Throughput and System Optimization

Autonomous driving depends on several systems operating together in real time.

The project will examine the full processing pipeline:

```text
Sensors / Cameras
        ↓
Perception
        ↓
Localization
        ↓
V2X / Environmental Information
        ↓
Path Planning
        ↓
Control
        ↓
Steering + Throttle
```

Development will focus on identifying and reducing unnecessary delays and computational bottlenecks throughout this pipeline.

Areas of interest include:

- Camera and sensor processing
- Detection latency
- V2X message processing
- Path-planning performance
- Control-loop performance
- Communication between subsystems
- CPU/GPU utilization
- Data-processing efficiency
- Overall end-to-end reaction time

The goal is to allow the QCar to process information and react to its environment more efficiently.

---

## System Architecture

The QCar autonomy stack can generally be separated into the following major subsystems:

### Perception

Processes camera and sensor information to understand the surrounding environment.

Examples include:

- Lane detection
- Object detection
- Traffic-sign detection
- Traffic-light detection
- Depth information

### Localization

Estimates the position, orientation, velocity, and movement of the vehicle.

Localization information may be produced using multiple sensors and estimation techniques, including sensor fusion and Extended Kalman Filtering.

### Planning

Determines where the vehicle should travel.

Planning may include:

- Map nodes
- Graph-based route planning
- Waypoint generation
- Shortest-path calculations
- Mission-specific routes

### Control

Converts the desired path and vehicle state into commands that control the physical vehicle.

Primary outputs include:

- Steering commands
- Throttle commands
- Speed control

### V2X

Provides additional information from connected infrastructure or vehicles that can influence the vehicle's planning and control decisions.

---

## 2027 Objectives

The primary objective of the 2027 development cycle is to produce a QCar that demonstrates:

- Reliable V2X communication
- Stable lane following
- Accurate and consistent object detection
- Smooth steering and vehicle control
- Appropriate responses to detected objects
- Reduced processing delays
- Improved subsystem coordination
- Repeatable autonomous driving behavior
- Improved overall system efficiency

---

## Development Approach

Development should prioritize understanding and measuring existing behavior before making major changes.

The general process should be:

1. Understand the existing implementation.
2. Establish baseline vehicle behavior.
3. Identify specific weaknesses or bottlenecks.
4. Modify one subsystem or behavior at a time.
5. Test changes in controlled conditions.
6. Compare results against the baseline.
7. Document results.
8. Integrate successful improvements into the larger system.

Changes affecting vehicle motion should be tested carefully and incrementally.

---

## Current Focus

Current development is centered around understanding and improving the existing physical QCar implementation, including:

- Road-map and waypoint generation
- Graph-based route planning
- Mission execution
- Speed control
- Steering control
- Perception and detection configuration
- V2X behavior
- Overall autonomy-stack performance

Additional areas will be investigated as testing identifies new performance limitations.

---

## Testing

Changes should be tested at multiple levels whenever possible.

### Software-Level Testing

Verify:

- Algorithms execute correctly
- Inputs and outputs are valid
- Communication remains stable
- Exceptions are handled correctly
- Changes do not break existing functionality

### Vehicle-Level Testing

Evaluate:

- Lane-following stability
- Steering behavior
- Speed consistency
- Detection reliability
- V2X response
- Mission completion
- Recovery behavior
- Processing latency

### Performance Testing

Where possible, record:

- Frame rate
- Detection processing time
- Control-loop frequency
- V2X message latency
- CPU/GPU usage
- End-to-end response time

Performance measurements should be used to identify where optimization work provides the greatest benefit.

---

## Safety

The QCar is a physical autonomous vehicle and should be tested carefully.

Before testing changes that affect vehicle movement:

- Verify steering and throttle limits.
- Confirm emergency-stop functionality.
- Begin testing at low speeds.
- Keep the vehicle within a controlled test area.
- Avoid introducing multiple major control changes simultaneously.
- Monitor the vehicle during all physical testing.

---

## Documentation

Significant changes should include documentation describing:

- What was changed
- Why the change was made
- How the change was tested
- Results of testing
- Known limitations
- Recommended next steps

Clear documentation will allow future team members to understand both the software and the reasoning behind previous development decisions.

---

 ## Project Goal

The long-term goal of the 2027 QCar project is to create a more dependable, responsive, and efficient autonomous vehicle platform by improving the interaction between perception, localization, V2X communication, planning, and control.

Development will focus not only on improving individual algorithms, but also on improving how information moves through the complete autonomy pipeline. The final system should be capable of detecting and interpreting its environment, communicating with surrounding infrastructure and vehicles, following lanes and planned routes accurately, and making timely control decisions with minimal unnecessary processing delay.

The 2027 development cycle will therefore emphasize measurable improvements in autonomy performance, system reliability, processing throughput, and repeatability during physical vehicle testing.
