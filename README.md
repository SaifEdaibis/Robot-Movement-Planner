# Robot Movement Planner

I built this project to explore how robotic arms can plan and execute movement around obstacles. It is a Python application that simulates a three-joint robotic arm in a 2D environment using Pygame.

The program lets the user choose a starting position and a target position for the robot and place or remove square obstacles in the environment. Once a target is selected, the program calculates a set of joint angles that can reach the target and searches for a path between the starting and ending configurations.

The pathfinding system uses a custom implementation of A*. Instead of searching directly through the robot's position, the algorithm searches through possible combinations of the robot's three joint angles. Each combination is represented as a node, and neighbouring nodes are created by making small changes to the joint angles. The algorithm uses a priority queue to determine which configuration to explore next.

Before a configuration can be used, the program checks whether the robot would collide with an obstacle. The collision system checks the robot's arms and joints and also uses additional padding so that the robot does not move directly against an obstacle.

The robot's joint positions are calculated using trigonometry and forward kinematics. Inverse kinematics is used to determine suitable joint angles for reaching a selected position. The robot then follows the calculated route by making small changes to its joint angles.

One of the main problems I encountered was the amount of time the pathfinder could take when there was no possible route. I improved this by using Python's `heapq` module for the priority queue and adding conditions that allow the search to stop when a valid route cannot be found.

The project is split across several Python files so that the robot, pathfinding system, obstacles, display, controls, and environment can be worked on separately. I used object-oriented programming for the main components of the program.

## Running the project

The project was developed using Python 3.13 and Pygame 2.6.1.

Clone the repository and enter the project folder:

```powershell
git clone https://github.com/SaifEdaibis/Robot-Movement-Planner.git
cd Robot-Movement-Planner
```

Create a virtual environment using Python 3.13:

```powershell
py -3.13 -m venv .venv
```

Activate the environment:

```powershell
.venv\Scripts\activate
```

Install the required package:

```powershell
python -m pip install -r requirements.txt
```

Run the program:

```powershell
python main.py
```

## Project files

`main.py` starts the application and connects the different parts of the program.

`robot.py` contains the robot, its joints, movement, angle calculations, and parts of the node system used by the pathfinder.

`path.py` contains the main path-planning system and handles finding routes between the starting and ending configurations.

`obstacle.py` contains the obstacle objects used in the environment.

`world.py` manages the environment and the objects within it.

`controller.py` handles user controls and input.

`display.py` contains the Pygame display and drawing functionality.

`Arcs.py` contains the classes used to display the robot's joint angle information.

`settings.py` contains configuration values used throughout the project.

`requirements.txt` contains the external Python dependencies required to run the project.

## What I learned

This project was mainly an exercise in applying programming to a robotics problem. While building it, I learned more about object-oriented Python, Pygame, Git and GitHub, trigonometry, forward and inverse kinematics, A* pathfinding, collision detection, and working with a larger codebase.

A large part of the project was also figuring out how to debug and improve an algorithm when it worked for simple cases but became slow or failed when a solution did not exist.
