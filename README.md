# Robot Movement Planner

This is a Python project I built to explore robotic arm movement and path planning. It simulates a three-joint robotic arm in a 2D environment using Pygame.

The program allows the user to control the robot's joint angles, place and move obstacles, and choose a starting and ending position for the robot. The path planner then calculates the joint configurations needed to move the robot between the two positions while avoiding the obstacles.

The pathfinding system uses a custom implementation of A*. Instead of searching through positions on the screen, the algorithm searches through possible combinations of the robot's three joint angles. Each combination is represented as a node. The algorithm generates neighbouring configurations by changing one joint angle at a time and uses a priority queue to decide which configuration to explore next.

Before a configuration is accepted, the program checks whether the robot would collide with any obstacles. The collision checking accounts for both the robot's arms and joints and includes additional padding around them so that the robot maintains some distance from obstacles.

The robot's joint positions are calculated using trigonometry and forward kinematics. Inverse kinematics is used to find suitable joint angles for reaching the selected ending position. Once a route has been found, the robot moves through the calculated configurations by gradually changing its joint angles.

One of the main problems I encountered was the amount of time the pathfinder could take when there was no possible route. I improved this by using Python's `heapq` module to manage the priority queue and adding conditions that allow the search to stop when a valid route cannot be found.

The project is split into several files so that different parts of the program can be developed separately. The main components are written using classes, including the robot, path planner, obstacles, world, display, and controllers.

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

`robot.py` contains the robot, its joint positions and angles, movement, kinematics calculations, and the node functions used by the pathfinding system.

`path.py` contains the main path planner and the calculations used to find suitable starting and ending configurations.

`obstacle.py` contains the obstacle class and handles obstacle movement, boundaries, and collisions between obstacles.

`world.py` manages the objects in the environment, including the robot, obstacles, controllers, and path icons.

`controller.py` handles user input and the different controls used to interact with the robot and path planner.

`display.py` contains the main Pygame display functionality.

`Arcs.py` contains the angle arc used to display the robot's joint angles.

`settings.py` contains the configuration values used throughout the project.

`requirements.txt` contains the external Python packages required to run the project.

## What I learned

I built this project to apply programming concepts to a robotics problem rather than working with small standalone programs. While developing it, I learned more about object-oriented Python, Pygame, Git and GitHub, trigonometry, forward and inverse kinematics, A* pathfinding, collision detection, and working with a larger codebase.

The project also gave me experience debugging algorithms and improving their performance. In particular, I had to deal with situations where a path did not exist and the search could continue for a long time without finding a solution.
