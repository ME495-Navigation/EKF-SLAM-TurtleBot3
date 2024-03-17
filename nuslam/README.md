# NUSLAM Library
A library for implementing EKF SLAM on the Turtlebot.

# Launchfile
Use `ros2 launch nuslam turtlebot_bringup.launch.xml` on the turtlebot
to launch the required nodes to run EKF SLAM using Unknown Data Association.

Use `ros2 launch nuslam pc_bringup.launch.xml` on your computer to visualise the
data from the real robot in Rviz.

Use `ros2 launch nuslam slam.launch.xml cmd_src:=teleop robot:=nusim use_rviz:=true`
to launch the required nodes to control the turtlebot in simulation (using Fake Sensor Data).

Use `ros2 launch nuslam unknown_data_assoc.launch.xml cmd_src:=teleop robot:=nusim use_rviz:=true`
to launch the required nodes to control the turtlebot in simulation (using Unknown Data Association).

Use `ros2 launch nuslam landmark_detect.launch.xml cmd_src:=teleop robot:=nusim use_rviz:=true`
to launch landmark detection test simulation.

## Demo Video (Real Robot)
https://private-user-images.githubusercontent.com/72541517/313494955-4dafcb6f-c8b4-4a2a-94d1-569984650a81.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MTA2OTg5NTMsIm5iZiI6MTcxMDY5ODY1MywicGF0aCI6Ii83MjU0MTUxNy8zMTM0OTQ5NTUtNGRhZmNiNmYtYzhiNC00YTJhLTk0ZDEtNTY5OTg0NjUwYTgxLm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNDAzMTclMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjQwMzE3VDE4MDQxM1omWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTZiNGViOThjNjVhNmYxY2EzYjQwODI5YzBkNDJmZjE2ZTlhNzlkYmU3OGM1M2MwMzVkMTNmYzVkNGIyOWE3NTQmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JmFjdG9yX2lkPTAma2V5X2lkPTAmcmVwb19pZD0wIn0.6zfE7xP9qa2sk-2agXbUtrUZ3mxLK7-eDK_DgP7BSDk

### Results (For EKF SLAM with Landmark Detection and Unknown Data Association)
##### Ground Truth
```
x: 0.0 m
y: 0.0 m
θ: 0.0 rad
```

##### Odometry Estimate (Blue)
```
x:  -0.0144 m
y:  -0.3531 m
θ: 0.1169 rad

Positional error: 0.353 m
Rotational error: 0.116 rad

```

##### EKF SLAM Estimate (Green)
```
x:  0.0289 m
y:  0.0077 m
θ:  0.0105 rad

Positional error: 0.029 m
Rotational error: 0.096 rad
```

## Demo Video (Simulation)

https://private-user-images.githubusercontent.com/72541517/312706714-25da5848-e1b6-4554-9eb9-b78f0942ac20.webm?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MTAzOTMxNjksIm5iZiI6MTcxMDM5Mjg2OSwicGF0aCI6Ii83MjU0MTUxNy8zMTI3MDY3MTQtMjVkYTU4NDgtZTFiNi00NTU0LTllYjktYjc4ZjA5NDJhYzIwLndlYm0_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjQwMzE0JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI0MDMxNFQwNTA3NDlaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT05NTU4ZGRlZWM1MWE2ZWEyM2RkNjdiYWNjNGVlN2EwZDZhYjgyNWYwZDkwMTdkYTUxYjMzMzEwOTE1ZjgyZmZkJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZhY3Rvcl9pZD0wJmtleV9pZD0wJnJlcG9faWQ9MCJ9.Bvp-EbtEdhk1Px7aOZqCecJid2rSvTNs0UTO4gwbP8Q

### Results (For EKF SLAM with Landmark Detection and Unknown Data Association)
##### Ground Truth
```
x: 0.0 m
y: 0.0 m
θ: 0.0 rad
```

##### Odometry Estimate (Blue)
```
x:  0.0147 m
y:  -0.9105 m
θ: 0.0491 rad

Positional error: 0.910 m
Rotational error: 0.049 rad

```

##### EKF SLAM Estimate (Green)
```
x:  0.0122 m
y:  0.0073 m
θ:  0.0964 rad

Positional error: 0.014 m
Rotational error: 0.096 rad
```

## SLAM Example (Simulation with Fake Sensor Data)

![](images/SLAM_example.png)
