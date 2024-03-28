This is a repository to use a turtleBot3 Burger to solve tasks provided by Royston. Tasks includes navigating around the maze, sending a HTTP requests to the ESP32 to open one of two
doors, shooting 5 ping-pong balls into a bucket, and completely map the maze.

Software

Here are the steps of the robot is going to take:
1) The turtleBot3 will navigate around the maze using a frontier algorithm and Astar Pathfinding algorithm.
2) Since the coordinates of the endpoint is given, the turtleBot3 will send a HTTP requests to an ESP32 through someone's mobile data. The ESP32 will then open one of the doors.
3) The turtleBot3 will line follow its way to the bucket, to which it will shoot 5 ping-pong balls into the bucket.
4) The turtleBot3 will then line follow its way back to the exit, and continue to map the maze until either the time is finished, or it has finished mapping.

Electrical

We will use a total of 4 electrical components inside our turtleBot3, which includes:
1) x2 servomotors
2) x2 IR sensors

Mechanical

We plan to make a catapult that stores 5 ping-pong balls, and fling them into the bucket.

NOTE: Read "README_software.md" to learn more about the programs we will be using inside this repository.
