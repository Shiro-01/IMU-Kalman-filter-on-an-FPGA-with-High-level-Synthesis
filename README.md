# IMU Kalman Filter on FPGA using High-Level Synthesis

## Overview

This repository contains the work and documentation for my Bachelor Thesis, which focuses on the implementation of IMU sensor fusion using a Kalman filter on an FPGA platform using High-Level Synthesis (HLS).

The goal of the project is to investigate how sensor fusion algorithms for Inertial Measurement Units (IMUs) can be efficiently implemented on FPGA hardware using modern open-source toolchains and high-level design methods.

The project explores the full pipeline starting from sensor data acquisition, through algorithm development and validation in software, to hardware acceleration on FPGA.

## WBS

```mermaid
---
title: Work Breakdown Structure
---
stateDiagram-v2
direction LR

    Thesis_Submission --> Orientation_Research_Phase
    Thesis_Submission --> System_Design_Phase
    Thesis_Submission --> Implementation_Phase
    Thesis_Submission --> Evaluation_Phase


%% ---------------- Orientation / Research ----------------
    Orientation_Research_Phase --> Thesis_Guidelines_Review
    Orientation_Research_Phase --> Requirements_Analysis
    Orientation_Research_Phase --> IMU_Research
    Orientation_Research_Phase --> Sensor_Fusion_Algorithm_Research
    Orientation_Research_Phase --> FPGA_Toolchain_Familiarization
    Orientation_Research_Phase --> Documentation

    Documentation --> GitHub
    Documentation --> Notion_Project_Management
    Documentation --> Overleaf_Thesis_Document


%% ---------------- System Design ----------------
    System_Design_Phase --> System_Architecture_Design
    System_Design_Phase --> IMU_Interface_Design(Drivers)
    System_Design_Phase --> Kalman_Filter_Model_Design
    System_Design_Phase --> FPGA_Data_Interface_Design(I2C_FSM_Diagram)

%% ---------------- Implementation ----------------
    Implementation_Phase --> ESP32_IMU_Data_Acquisition
    Implementation_Phase --> IMU_Data_Logging
    Implementation_Phase --> Cpp_Kalman_Filter_Implementation
    Implementation_Phase --> Python_Data_Visualization
    Implementation_Phase --> FPGA_I2C_Interface_IP
    Implementation_Phase --> HLS_Kalman_Filter_Conversion


%% ---------------- Evaluation ----------------
    Evaluation_Phase --> IMU_Benchmarking_and_Selection
    Evaluation_Phase --> Sensor_Fusion_Algorithm_Comparison
    Evaluation_Phase --> Performance_Evaluation

    Performance_Evaluation --> ESP32_Software_Implementation
    Performance_Evaluation --> FPGA_HLS_Implementation
    Performance_Evaluation --> Commercial_IMU_Comparison```
```
