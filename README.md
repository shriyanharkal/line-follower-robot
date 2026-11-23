# Line Follower Robot

## Overview
A robot that follows a black line on a white surface using IR sensors and PD control.

## Hardware
- Arduino Uno R3
- 3x TCRT5000 IR sensors
- TB6612FNG Motor driver module
- 2x DC motors
- Chassis
- 2x 18650 Batteries

## How It Works
- Arduino reads an analog signal from 0 - 1023. Each sensor has its own range of readings. So we normalize the values so they are comparable. This requires min and max values for each sensor to find its range. This happens in `calibrate()` through `setup()`.
- We use the normalized values and weights to calculate a weighted average to find the position of the line. Weights are used since each sensor's position relative to the middle determines how much it contributes to the calculated position.
- Once we know the position, we can use it as our error since the ideal position should be 0. We use PD control to correct the error by steering the robot. Steering is done by varying the motor speeds on the left and right sides.

## Results
The robot follows my track successfully more than 30 consecutive times. Each lap takes approximately 10 seconds.

Using only 3 sensors limited the robot's detection of the line's position and strength, which made it incapable of taking an immediate 90-degree turn.

## Media

### Photos (Top and bottom of the robot)
<img width="1280" height="960" alt="Topside" src="https://github.com/user-attachments/assets/864df484-b4f4-400e-b51d-4259c0e46e32" />
<img width="1280" height="960" alt="Bottomside" src="https://github.com/user-attachments/assets/df7f0827-8346-4e3e-a754-55bf29a22208" />

### Video
[Watch the robot in action](https://youtu.be/MOGDNYaP8P0)

## Challenges & Improvements
- Finding a reliable and functioning sensor height took some experimenting. I had to space out sensors to reduce crosstalk as well.
- Right motor was slower than the left motor for the same speed. This required slowing down the left motor. I chose to slow down the left motor since it would consume less power than speeding up the right motor.
- Tuning required lots of experimentation to find good values for proportional gain and derivative gain.
