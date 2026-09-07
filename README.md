# Robot Movement Planner

This is a Python project I built to experiment with robotic arm movement, path planning, and collision detection. The program simulates a three-joint robotic arm in a 2D environment and allows the user to choose where the robot should start and where it should move.

The user can place and remove square obstacles in the environment and then choose a target position. The program calculates the joint angles needed to reach the target and uses a custom A* pathfinding algorithm to find a route that avoids the obstacles.

The robot uses forward and inverse kinematics to calculate the positions of its joints. The path planner works with different combinations of joint angles rather than simply moving the robot directly toward the target. Each possible configuration is treated as a node, and the A* algorithm searches through these configurations to find a valid route.

Collision detection is performed while the path is being calculated. If a configuration causes one of the robot's arms or joints to intersect with an obstacle, that configuration is rejected. I also added padding around the robot and obstacles so that the robot does not move directly against an obstacle.

One of the main problems I ran into was the amount of time the pathfinding could take when there was no possible route. I improved this by using Python's `heapq` priority queue and adding early exits when the algorithm determines that a route cannot be found.

The project is written in Python and uses Pygame for the interface and visualization. I used object-oriented programming to separate the robot, path planner, obstacles, display, controller, and world into different classes and files.

## Running the project

Clone the repository and enter the project folder.

```bash
git clone https://github.com/SaifEdaibis/Robot-Movement-Planner.git
cd Robot-Movement-Planner
```

Create a virtual environment and install the required packages.

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

The program can then be started with:

```bash
python main.py
```

## Files

`main.py` starts the program and brings the different parts of the project together.

`robot.py` contains the robotic arm, its joint angles, movement, and kinematics calculations.

`path.py` contains the path planning system and the A* search.

`obstacle.py` handles the obstacles used in the environment.

`world.py` manages the environment in which the robot operates.

`controller.py` handles user input and controls.

`display.py` contains the Pygame display functionality.

## What I learned

I built this project to get more comfortable with Python and to apply programming concepts to a robotics problem. While working on it, I learned more about object-oriented programming, Git and GitHub, Pygame, trigonometry, forward and inverse kinematics, A* pathfinding, and collision detection.

The project also gave me experience debugging a larger codebase and dealing with problems where an algorithm works in some situations but becomes extremely slow or fails when no valid solution exists.
