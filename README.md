# Autonomous Zumo Robot Navigation

**Authors:** Amir Amber & Shay Saraf  
**Hardware:** Pololu Zumo 32U4  

## 📖 Project Overview

This repository contains the source code and analysis tools for an autonomous control system designed for the Zumo 32U4 robot. The project successfully implements a complex mission profile requiring the integration of distinct navigational behaviors: precision line following, geometric corridor navigation (wall bouncing), and object manipulation.

The system utilizes a **Finite State Machine (FSM)** to switch between Open-Loop and Closed-Loop control paradigms, allowing the robot to navigate a multi-stage obstacle course and return to its starting position autonomously.

## 🚀 Key Features & Stages

The robot logic is divided into 5 distinct stages managed by the FSM:

1.  **Stage 1: Line Following (PD Control)**
    * Utilizes a Proportional-Derivative (PD) controller ($K_p=4, K_d=6$) to navigate curved paths.
    * Uses a 5-channel sensor array for error calculation.

2.  **Stage 2: Corridor Navigation (Reactive)**
    * Navigates a corridor without a central guide line.
    * Implements a "Wall Bounce" logic: the robot cruises until a border is detected, triggering an immediate corrective steer.

3.  **Stage 3: Object Manipulation (Hybrid)**
    * **Detection:** Uses proximity sensors (Closed-Loop).
    * **Action:** Executes a timed "Push and Reverse" maneuver (Open-Loop) to clear obstacles from the path.

4.  **Stage 4: Search & Finish**
    * Resumes line following after obstacle clearance.
    * Detects the final stopping point using proximity sensors.

5.  **Stage 5: Return to Home**
    * Executes a blind ~140-degree turn.
    * Navigates the course in reverse to return to the start coordinates $(0,0)$.

## 📂 Repository Structure

* **`main.ino`**: The entry point for the Arduino application. Contains the main loop, FSM logic (switch-case for stages), and sensor initialization.
* **`ZumoController.cpp` / `ZumoController.h`**: A custom controller class that encapsulates:
    * Odometry tracking (calculating $X, Y, \theta$ based on encoder data).
    * Motor control abstraction.
    * State variables.
* **`Zumo32U4.h`**: Main header file for the standard Pololu Zumo library.
* **`zumo_analysis.m`**: A MATLAB script used to process telemetry data transmitted via Serial. It generates the path tracking (Odometry), velocity, and error plots found in the project report.

## 🛠️ Hardware & Dependencies

* **Robot:** Pololu Zumo 32U4 (OLED version).
* **Sensors Used:**
    * Line Sensor Array (5 sensors).
    * Front Proximity Sensors.
    * Quadrature Encoders (for Odometry).
    * Buzzer (for auditory feedback).
* **Software:**
    * Arduino IDE.
    * [Zumo32U4 Arduino Library](https://github.com/pololu/zumo-32u4-arduino-library).

## 📊 Telemetry & Analysis

The robot transmits telemetry data via Serial at 100Hz. The `zumo_analysis.m` script visualizes this data to validate performance:
* **Path Reconstruction:** Plots estimated X vs Y position.
* **Velocity Profiles:** Visualizes PWM signals and PD corrections.
* **Error Analysis:** Graphs the PID error term to demonstrate convergence.

## 👥 Contributors

* **Amir Amber**
* **Shay Saraf**

