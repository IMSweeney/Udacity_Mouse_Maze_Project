This project was created as the Capstone project for the [Udactiy Data Scientist](https://www.udacity.com/course/data-scientist-nanodegree--nd025) course. The accompanying blog post can be found at a Github Pages site [here](https://imsweeney.github.io/Udacity_Mouse_Maze_Project/).

Path planing has become one of the forefront domains in artificial inteligence. One common application is for the navigation of robots through a maze, such as in [Micromouse](https://en.wikipedia.org/wiki/Micromouse) competitions. To make this possible there are several components. The first, environment sensing, involves the input of data about the environment through physical (or virtual) sensors. The second, SLAM, or simultanious localization and mapping involves the use of this sensor data to create a map of a given area while also maintaining some knowledge of the robot's position. The final piece is the path planning itself, generally an algorithm using a map of the environment and the location of the goal to plan the next best move.

This project is simulated micromouse competition. A pre-generated maze is provided as well as a few python files to give a framework in which the robot can operate.

Lasly a template robot inteligence is provided in: `robot.py`. The majority of the code written for this project is here. It contains an algorithm for mapping, pathing, and drawing the maze from the sensor data.

A much more detailed statement of this project can be found [here](https://docs.google.com/document/d/1ZFCH6jS3A5At7_v5IUM5OpAXJYiutFuSIjTzV_E-vdE/pub)

This project consisted of desinging a simple robot AI to navigate a maze. This robot had to perform 5 main functions to succed at this task:
1. Maintain an accurate record of it's current location
2. Create a map of the environment from the given sensor data
3. Navigate towards the goal
4. Exploration enough to find the optimal path

It was scored based on a combination of the time it spent exploring and the time for the best path. To inplement this AI, I Gathered sensor data into a graph structure containing nodes for each square in the maze and edges between them. Then I used a heuristic based search to determine the next square to navigate towards. Finally, I tuned the weights in this heuristic search until an acceptable score was achieved.

These were my final results:
| Size  | Optimal Path | Goal Score | My Score     |
| ----- | ------------ | ---------- | ------------ |
| 12x12 | 17           | 19.4       | **21.0**     |
| 14x14 | 23           | 26.3       | **28.0**     |
| 16x16 | 25           | 29.3       | **32.4**     |

For a detailed walkthrough of the project, see the Github Pages site [here](https://imsweeney.github.io/Udacity_Mouse_Maze_Project/).

The project used *numpy* for mathematics operations, and *pandas* for saving data to a csv and processing it. It also used the *Queue* module for implementing a breadth-first search algorithm, and *Graphics* for drawing the Maze and Robot. Lastly the *tqdm* and *subprocess* modules were used to automate the testing of hyper-parameters of the model.

Included files:

* images
  * 12x12_best_path.PNG: A visualization of the optimal path through the 12x12 maze
  * 12x12_frontier.PNG: A visualization of the frontier nodes midway through a run
  * 12x12_start_goal.PNG: A visualization of the maze made with showmaze.py
  * 16x16_best_path.PNG: A visualization of the optimal path through the 16x16 maze
* AI_startercode.zip: An archive of the original project code for reference
* README.md: This document
* \_config.yml: A simple theme for the Github Pages site
* index.md: The markdown document for the Github Pages site
* maze.py: Loads a maze text file into memory and checks its validity.
* parameter_tuning.py: Script for gathering and analyzing data for tuning the model.
* parameters.csv: The data store for each run, parameters used, and scores
* robot.py: Algorithms for mapping, planning, and drawing the maze
* showmaze.py: Uses the turtle module to draw a .txt maze
* test_maze_01.txt: Starter 12x12 maze in .txt form
* test_maze_02.txt: Starter 14x14 maze in .txt form
* test_maze_03.txt: Starter 16x16 maze in .txt form
* tester.py: Tests the robot in a maze
