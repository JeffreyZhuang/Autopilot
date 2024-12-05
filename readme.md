# Autopilot

![](autopilot.webp)
![](mainboard.webp)

## File Structure

## Software Architecture

## Coordinate system

X is in the direction opposite to the LED. Following right hand rule. Clockwise rotation when facing in the direction of the axis of rotation.
North (x), East (y), Down (z), Roll (x), Pitch(y), Yaw (z) 

![](coordinate_system.png)

## To-Do List
1. Send AHRS data over USB and visualize to analyze filter
3. Speed up micro sd card
4. Micro sd card bug when placed after i2c_begin
5. Position Kalman filter
6. Add magnetometer to AHRS
7. Python GUI with graphs for visualizing USB data
8. Telemetry
9. GPS
10. GCS
11. Servos
12. Failsafe
13. Battery voltage and current sensing
14. Gravity compensation for IMU
15. PID controllers
16. Path planning
17. State machine
18. Move Autopilot folder out of lib folder