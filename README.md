# Udacity_Mouse_Maze_Project
Project for Udacity Data Analyst Capstone.

# Project Definition
## Project Overview
*Student provides a high-level overview of the project. Background information such as the problem domain, the project origin, and related data sets or input data is provided.*

Path planing has become one of the forefront domains in artificial inteligence. One common application is for the navigation of robots through a maze, such as in [Micromouse]https://en.wikipedia.org/wiki/Micromouse competitions. To make this possible there are several components. The first, environment sensing, involves the input of data about the environment through physical (or virtual) sensors. The second, SLAM, or simultanious localization and mapping involves the use of this sensor data to create a map of a given area while also maintaining some knowledge of the robot's position. The final piece is the path planning itself, generally an algorithm using a map of the environment and the location of the goal to plan the next best move.

This project is simulated micromouse competition. A pre-generated maze is provided as well as a few python files to give a framework in which the robot can operate.

* '''maze.py''': Loads a maze text file into memory and checks its validity.
* '''showmaze.py''': Uses turtle to draw a .txt maze
* '''tester.py''': Tests the robot in a maze

Lasly a template robot inteligence is provided in: '''robot.py'''. All of the code written for this project is here. It contains an algorithm for mapping, pathing, and drawing the maze from the sensor data.




## Project Questions
### Data Explorations
Use the robot specifications section to discuss how the robot will interpret and explore its environment. Additionally, one of the three mazes provided should be discussed in some detail, such as some interesting structural observations and one possible solution you have found to the goal (in number of steps). Try to aim for an optimal path, if possible!

Using sensor data the robot creates a graph of nodes where a node is connected to another node by an edge if the path between those two nodes is valid. During the first run pathing is determined by chosing the unexplored node with the best explore score. This score is defined by a combination of the distance to the goal and the distance from the current location of the bot. Once a node is chosen to explore, a path is created from the current location to this new goal using a simplified A* algorithm. Once the goal is found, this score is modified to only use the distance from the current node to allow additional exploration before the second run. For the second run the robot uses this same simplified A* to navigate straight to the goal.


### Exploratory Visualization
This section should correlate with the section above, in that you should provide a visualization of one of the three example mazes using the showmaze.py file. Your explanation in Data Exploration should coincide with the visual cues from this maze.


### Benchmark
You will need to decide what you feel is a reasonable benchmark score that you can compare your robot’s results on. Consider the visualization and data exploration above: If you are allowed one thousand time steps for exploring (run 1) and testing (run 2), and given the metric defined in the project, what is a reasonable score you might expect? There is no right or wrong answer here, but this will help with your discussion of solutions later on in the project.

For test_maze_01 (the 12x12 maze) the shortest possible path is 17 moves. Therfore, the best possible score would be 17 / 30 (for the first run) + 17 (for the second run), for a best score of **17.6**. If approximately 50% of the cells need to be explored to find the best path this would correlate to 12 x 12 / 2 = 72 cells to explore, which would take 72 moves minimally. The score for this would be 72 / 30 + 17 = **19.4**. This robot algorithm was able to score **22.0**.


### Data Preprocessing
Because there is no data preprocessing needed in this project (the sensor specification and environment designs are provided to you), be sure to mention that no data preprocessing was necessary and why this is true.


### Free-Form Visualization
Use this section to come up with your own maze. Your maze should have the same dimensions (12x12, 14x14, or 16x16) and have the goal and starting positions in the same locations as the three example mazes (you can use test_maze_01.txt as a template). Try to make a design that you feel may either reflect the robustness of your robot’s algorithm, or amplify a potential issue with the approach you used in your robot implementation. Provide a small discussion of the maze as well.


### Improvement
Consider if the scenario took place in a continuous domain. For example, each square has a unit length, walls are 0.1 units thick, and the robot is a circle of diameter 0.4 units. What modifications might be necessary to your robot’s code to handle the added complexity? Are there types of mazes in the continuous domain that could not be solved in the discrete domain? If you have ideas for other extensions to the current project, describe and discuss them here.

In a continuous domain