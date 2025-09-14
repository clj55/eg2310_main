This repository contains the software portion of the entire system design used in a turtleBot3 Burger. Tasks include autonomously navigating around an unknown maze, sending a HTTP requests to the ESP32 to open one of two doors, shooting 5 ping-pong balls into a bucket, completely mapping the maze.

Operational Flow:
1) The turtleBot3 navigates through the maze using line-following and stops between 2 doors. 
2) The turtleBot3 then sends a HTTP request to an ESP32, unlocking one of the doors. It then continues into the room stopping in front of a bucket. 
3) After shooting 5 ping pong balls into the bucket using a catapult, the robot exits the room. 
4) The turtleBot3 navigates around the maze using a gap-based exploration algorithm mapping the remaining parts of the maze in the process.


<img width="651" height="316" alt="Screenshot from 2025-09-14 16-09-15(1)" src="https://github.com/user-attachments/assets/7c2a0abb-2d23-4c30-8861-adea37a644b6" />

Main Control: finalcontrol.py

Autonomous Exploration: clean.py
