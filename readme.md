# Autopilot

![](autopilot.webp)
![](mainboard.webp)

## File Structure

## Software Architecture

## Coordinate system

X is in the direction opposite to the LED. Following right hand rule. Clockwise rotation when facing in the direction of the axis of rotation.

![](coordinate_system.png)

## To-Do List
1. Send AHRS data over USB and visualize to analyze filter
2. Rotate IMU and AHRS in correct coordinate system
3. Speed up micro sd card
4. Micro sd card bug when placed after i2c_begin
5. Position Kalman filter
6. Add magnetometer to AHRS