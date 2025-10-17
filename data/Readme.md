# Documentation for UAV Digital Twin Multimodal Dataset

## 1. Dataset Overview

This dataset is designed to support UAV digital twin modeling and virtual-real data synchronization verification. It includes multimodal sensor data, error data, and preprocessed training data from both the physical UAV (real UAV) and its digital twin. Covering typical UAV flight phases, the dataset can be used to verify the consistency of virtual and real data and optimize the accuracy of the digital twin model.

## 2. Data Modal Description

The dataset is divided into two core modals based on flight phases, distinguished by identifier fields in filenames:

- **Hover Mode**: Identified as `mode2_6`It contains sensor data of the UAV in a stable hovering state. The typical scenario is hovering at a height of 0.5-1.5 meters above the ground. The data characteristics reflect the sensor noise properties under low dynamic disturbance.
- **Takeoff Mode**: Identified as `mode2_3`It contains dynamic process data of the UAV from ground startup to climbing to a stable height, covering phases such as acceleration, attitude adjustment, and altitude increase. The data characteristics reflect the sensor response properties in dynamic scenarios.

## 3. File Naming Rules and Data Types

All dataset files are in CSV format, with their sources and purposes identified by prefixes. The details are as follows:

| Filename Prefix | Data Type                                  | Core Purpose                                                 |
| --------------- | ------------------------------------------ | ------------------------------------------------------------ |
| `*raw*.csv`     | Physical UAV Raw Dataset                   | Raw data collected by the on-board sensors of the physical UAV, reflecting the real state of the physical system. It can be used as a reference benchmark for the digital twin model. |
| `*sim*.csv`     | Digital Twin Dataset                       | Data output from the digital twin simulation, reflecting the virtual model’s simulation results of the physical system. It is used for comparative analysis with physical UAV data. |
| `*err*.csv`     | Error Dataset                              | Frame-by-frame errors between the physical UAV data and digital twin data, directly reflecting the deviation degree between the virtual and real models. It is used for error tracing and model optimization. |
| `*sycn*.csv`    | Virtual-Real Synchronized Training Dataset | Data preprocessed with time alignment and frequency matching, which can be directly used for model training. |

## 4. Detailed Explanation of Data Fields

All datasets include 18 dimensions of fields, covering sensor measurements and motion state quantities. The details are as follows:

| Field Name              | Physical Meaning                            | Unit       | Description                                                  |
| ----------------------- | ------------------------------------------- | ---------- | ------------------------------------------------------------ |
| `gyro_rad[0]`           | Gyroscope X-axis angular velocity           | rad/s      | Rotational angular velocity around the X-axis in the body coordinate system |
| `gyro_rad[1]`           | Gyroscope Y-axis angular velocity           | rad/s      | Rotational angular velocity around the Y-axis in the body coordinate system |
| `gyro_rad[2]`           | Gyroscope Z-axis angular velocity           | rad/s      | Rotational angular velocity around the Z-axis in the body coordinate system |
| `accelerometer_m_s2[0]` | Accelerometer X-axis acceleration           | m/s²       | Linear acceleration along the X-axis in the body coordinate system (including gravitational component) |
| `accelerometer_m_s2[1]` | Accelerometer Y-axis acceleration           | m/s²       | Linear acceleration along the Y-axis in the body coordinate system (including gravitational component) |
| `accelerometer_m_s2[2]` | Accelerometer Z-axis acceleration           | m/s²       | Linear acceleration along the Z-axis in the body coordinate system (including gravitational component) |
| `magnetometer_ga[0]`    | Magnetometer X-axis magnetic field strength | Gs (Gauss) | Magnetic field measurement along the X-axis in the body coordinate system |
| `magnetometer_ga[1]`    | Magnetometer Y-axis magnetic field strength | Gs         | Magnetic field measurement along the Y-axis in the body coordinate system |
| `magnetometer_ga[2]`    | Magnetometer Z-axis magnetic field strength | Gs         | Magnetic field measurement along the Z-axis in the body coordinate system |
| `x`                     | Position X-coordinate                       | m          | X-direction position in the ground coordinate system (origin at the takeoff point) |
| `y`                     | Position Y-coordinate                       | m          | Y-direction position in the ground coordinate system         |
| `z`                     | Position Z-coordinate (altitude)            | m          | Vertical altitude in the ground coordinate system (relative to the takeoff point) |
| `vx`                    | X-direction velocity                        | m/s        | Linear velocity along the X-axis in the ground coordinate system |
| `vy`                    | Y-direction velocity                        | m/s        | Linear velocity along the Y-axis in the ground coordinate system |
| `vz`                    | Z-direction velocity (climb/descent speed)  | m/s        | Vertical linear velocity in the ground coordinate system     |
| `ax`                    | X-direction acceleration (gravity-removed)  | m/s²       | Motion acceleration along the X-axis in the ground coordinate system |
| `ay`                    | Y-direction acceleration (gravity-removed)  | m/s²       | Motion acceleration along the Y-axis in the ground coordinate system |
| `az`                    | Z-direction acceleration (gravity-removed)  | m/s²       | Vertical motion acceleration in the ground coordinate system |

## 5. Sampling Frequency Description

- **Physical UAV Dataset (`\*raw\*.csv`)**: The sampling frequency is 50Hz, corresponding to the hardware output frequency of the on-board IMU (Inertial Measurement Unit).
- **Digital Twin Dataset (`\*sim\*.csv`)**: The sampling frequency is 100Hz, output by the virtual model at a fixed time step (0.01s), reflecting the advantage of higher temporal resolution of the digital twin body.
- **Synchronized Training Dataset (`\*sycn\*.csv`)**: The frequency has been unified to 50Hz through linear interpolation (for physical UAV data) and downsampling (for digital twin data), ensuring timestamp alignment and enabling direct use as model input.

## 6. Usage Recommendations

1. Model Accuracy Verification: It is recommended to compare the fields with the same name in `*raw*.csv` and `*sim*.csv`, and analyze the error distribution law of specific modals (e.g., takeoff phase) in combination with `*err*.csv`.
2. Training Data Selection: Use `*sycn*.csv` for model training to avoid time misalignment issues caused by frequency differences. If researching frequency adaptation algorithms, the original `*raw*.csv` and `*sim*.csv` can be used directly.