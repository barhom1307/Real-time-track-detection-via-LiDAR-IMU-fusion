# Real-time-track-detection-via-LiDAR-IMU-fusion

[![Video](/proj_images/video.jpg)](https://www.youtube.com/watch?v=cojekL9opk0)

"Real-time path detection for autonomous Formula vehicle" project was developed as a final project in Electrical Engineering Faculty, Technion haifa. 

The project was accompanied by [Vision and Image Sciences Laboratory (VISL)](https://visl.technion.ac.il/projects/2018w02/), and [Technion Formula Student Team](https://www.facebook.com/TechnionFSAE/).

Developed by Aviv Regev & Avinoam Barhom, Supervisor - Danny Veikhrman 

---

## Project Goals
1. Developing & Implementing cone identifier algorithm. 
2. IMU - Lidar integration.
3. Implementing route mapping based on IMU - Lidar joint output.
4. Server - Client network communication between car and user’s control App.
5. Implementing real time 3D graphic display on the server side.

## Project Logic Flow
![Image](/proj_images/sol_flow.png)

## System Design
![Image](/proj_images/sys_struct.png)

## Software Design
![Image](/proj_images/software_struct.png)

---

## Key processing steps:
1. Receiving suspect points 
2. IMU/Lidar integration
3. K-means clustering

![Image](/proj_images/stages.png)

--- 

## Error Measurements
![Image](/proj_images/error_measurement.png)



