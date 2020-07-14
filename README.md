# Udacity_Mouse_Maze_Project
Project for Udacity Data Analyst Capstone.
![Frontier16x16](/16x16_best_path.PNG)

# Project Definition
## Project Overview
*Student provides a high-level overview of the project. Background information such as the problem domain, the project origin, and related data sets or input data is provided.*

Path planing has become one of the forefront domains in artificial inteligence. One common application is for the navigation of robots through a maze, such as in [Micromouse](https://en.wikipedia.org/wiki/Micromouse) competitions. To make this possible there are several components. The first, environment sensing, involves the input of data about the environment through physical (or virtual) sensors. The second, SLAM, or simultanious localization and mapping involves the use of this sensor data to create a map of a given area while also maintaining some knowledge of the robot's position. The final piece is the path planning itself, generally an algorithm using a map of the environment and the location of the goal to plan the next best move.

This project is simulated micromouse competition. A pre-generated maze is provided as well as a few python files to give a framework in which the robot can operate.

* `maze.py`: Loads a maze text file into memory and checks its validity.
* `showmaze.py`: Uses turtle to draw a .txt maze
* `tester.py`: Tests the robot in a maze

Lasly a template robot inteligence is provided in: `robot.py`. All of the code written for this project is here. It contains an algorithm for mapping, pathing, and drawing the maze from the sensor data.

A much more detailed statement of this project can be found [here](https://docs.google.com/document/d/1ZFCH6jS3A5At7_v5IUM5OpAXJYiutFuSIjTzV_E-vdE/pub)

## Problem Statement
*The problem which needs to be solved is clearly defined. A strategy for solving the problem, including discussion of the expected solution, has been made.*

The goal of this project is to implement the create a robot AI that can navigate any maze provided and reach the goal. At each timestep, the robot will recieve sensor data from the tester and return it's choice of move. To do this the robot will have to do the following:

1. Maintain an accurate record of it's current location
2. Create a map of the environment from the sensor data
3. Navigate towards the goal
4. Do enough exploration to find the optimal path
5. *Bonus*: Create a visualization of the AI (chosen path, explored nodes)

I would expect the robot to be able to navigate any maze given if it has a valid path to the goal as well as to eventually find the optimal path. It should also create a visualization so that one can watch the robot to get an idea of how it is operating.

## Metrics
*Metrics used to measure performance of a model or result are clearly defined. Metrics are justified based on the characteristics of the problem.*

For this project the metric is stated by the competition as a score based on two runs of the maze. The first run is for exploration, while the second run is for reaching the goal as fast as possible.

$score = time steps in first run / 30 + time steps in second run$

The goal will be to minimize this score.

---

# Analysis
## Data Explorations / Visualizations
*Features and calculated statistics relevant to the problem have been reported and discussed related to the dataset, and a thorough description of the input space or input data has been made. Abnormalities or characteristics about the data or input that need to be addressed have been identified.*

Up to this point the maze and the robot have been theoretical. Now we will give them substance. The maze will be a grid of *n x n* squares, where *n* is an even number. There will be walls along the outside edge of the maze as well as many internal walls through which the robot cannot move. The robot will start on the bottom left corner of the maze and the goal will be any of the four squares in the center of the maze.

Here is the 12 x 12 maze as an example (from showmaze.py):
![12 x 12 maze](/12x12_start_goal.PNG)

As for the robot, it will occupy a single square and point in one of the four cardinal directions. It will also have three sensors on it's front, left, and right sides. These sensors will give the robot data about the walls. Specifically, each sensor will give an integer value for how many squares away a wall is in the direction of that sensor. In terms of movement, the robot can do two things on it's turn, move and rotate. It can rotate -90, 0, or 90 degrees clockwise, and it can move forwards or backwards up to three squares.

Let's take a look at that first example maze to get a better idea of what our robot is working with. The first thing that I notice is that the robot will have to move most (or even all) of the way to the right side of the maze to get to the goal. This means that any exploration done on the left half will be fairly useless. The next thing that I noticed is that there seem to be two main paths. One above the goal, and one below. The optimal path will be a variation of one of these. Perhaps the most interesting twist to this problem is that moves of three spaces and moves of one are weighted the same in terms of scoring. This would tend to reward path with fewer turns, rather that simply the shortest distance traveled. With all of this considered, I went to find the optimal path.

![12 x 12 best path](/12x12_best_path.PNG)

The pink squares are the moves that the robot will make (in this case in the optimal path to the goal). For this maze, the optimal path ended up being **17** moves long.


## Benchmark
To determine how efficient the robot is at mapping and path planning for these mazes it would be wise to develop a benchmark, or a score to shoot for.
Take the 12x12 maze for example. For this maze the shortest possible path is 17 moves. Therfore, the best possible score would be 17 / 30 (for the first run) + 17 (for the second run), for a best score of **17.6**. Since this would be an unrealistic goal (requiring zero wrong moves) we will assume some inefficiency in the exploration phase. If approximately 50% of the cells need to be explored to find the best path this would correlate to 12 x 12 / 2 = 72 cells to explore, which would take 72 moves minimally. The score for this would be 72 / 30 + 17 = **19.4**. This process can be similarly performed for each maze size. The results of this are summarized in this table:

| Size  | Optimal Path | Goal Score | My Score | Explore % |
| ----- | ------------ | ---------- | -------- | --------- |
| 12x12 | 17           | 19.4       | 22.0     | 0.7       |
| 14x14 | 23           | 26.3       | 31.5	   | 0.7       |
| 16x16 | 25           | 29.3       | 35.3	   | 0.7       |

# Methodology
## Data Preprocessing
*All preprocessing steps have been clearly documented. Abnormalities or characteristics about the data or input that needed to be addressed have been corrected. If no data preprocessing is necessary, it has been clearly justified.*

For this project the Data pre-processing was minimal. The only data that the robot recieves is the sensor data. This data is based on heading (forward, left, and right sensors) rather than absolute direction. For my purposes it was more valuable to have the data in absolute directions, so the sensor data is converted using the heading of the robot to absolute directions (e.g. left, up, down).

## Implementation
*The process for which metrics, algorithms, and techniques were implemented with the given datasets or input data has been thoroughly documented. Complications that occurred during the coding process are discussed.*

Now to discuss how the robot works. The robot has four main functions: 
1. Maintain an accurate record of it's current location
2. Create a map of the environment from the sensor data
3. Navigate towards the goal
4. Do enough exploration to find the optimal path

To perform each of these, a framework needed to be created to hold information about the maze. For this I chose to use a graph, where each node represents one of the cells of the maze. Edges between nodes represent valid, single-move paths between cells.

### Maintaing an Accurate Location
This is the simplest of the four by far, but it still has its challenges. The solution is to, at each timestep, use the previous sent move to update the current location of the robot. The challenge comes with invalid moves. If the robot were to update it's position when a move is rejected by the tester as invalid, it would be forever lost. This ends up being a non issue using the graph method to move, but with other methods each move would have to be first verified using the sensor data.

### Generating a Map
At each time step new information is gained through sensor data. This data is used to add nodes to the graph. For each sensor direction, paths are added for every valid move. For example, if the front sensors sees a wall 5 steps away, lets label the 5 cells in front of the robot *c1* through *c5*. There are 9 valid edges here:

*c1 - c2*, *c1 - c3*, *c1 - c4*

*c2 - c3*, *c2 - c4*, *c2 - c5*

*c3 - c4*, *c3 - c5*

*c4 - c5*

All of these edges would be added then added to the graph (if they do not already exist).

### Navigation
To navigate to a given node, a modified version of the breadth-first search algorithm was borrowed from this [tutorial](https://www.redblobgames.com/pathfinding/a-star/introduction.html) by Red Blob Games. Here is the algorithm as implemented in python:

```python
frontier = Queue()
frontier.put(self)
came_from = {}
came_from[self] = None

while not frontier.empty():
    current = frontier.get()
    for node in current.edges:
        if node not in came_from:
            frontier.put(node)
            came_from[node] = current

current = other
path = []
while current != self:
    path.append(current)
    current = came_from[current]
path.reverse()
return path
```

Since breadth-first search always finds the optimal path this algorithm guarantees that we take the best path between two nodes.

### Exploration
The breadth-first algorithm described in the previous section is good but it cannot explore new nodes, only plan a path to nodes have already been seen. To remedy this we need another path planning algorithm one level above this. This algorithm works as follows:
1. Each time a new node is added to the graph it is also added to a frontier set
2. When a node is visited it is removed from the frontier
3. If there is not currently a path being followed, choose the next frontier node to move to
4. Find the path to this node using the breadth-first algorithm above
5. move along this path

How this path is chosen is borrowed from heuristic search algorithms like A\*. Each node on the frontier is scored based on a combination of aspects. The wrinkle for this specific problem is that this combination changes depending on the phase of exploration that the robot is in. Firstly, there are two runs of the maze. This heuristic scoring only applies to the first, or exploration run, since the second run always has the goal cells as the node to path to. Secondly, the first run can be split up into two phases: The first pahse is for simply finding the goal. For ...

## Refinement
*The process of improving upon the algorithms and techniques used is clearly documented. Both the initial and final solutions are reported, along with intermediate solutions, if necessary.*

# Results
## Model Evaluation and Validation
*If a model is used, the following should hold: The final model’s qualities — such as parameters — are evaluated in detail. Some type of analysis is used to validate the robustness of the model’s solution.
Alternatively a student may choose to answer questions with data visualizations or other means that don't involve machine learning if a different approach best helps them address their question(s) of interest.*

## Justification
*The final results are discussed in detail.
Exploration as to why some techniques worked better than others, or how improvements were made are documented.*

# Conclusion
## Reflection
*Student adequately summarizes the end-to-end problem solution and discusses one or two particular aspects of the project they found interesting or difficult.*

## Improvement
*Discussion is made as to how at least one aspect of the implementation could be improved. Potential solutions resulting from these improvements are considered and compared/contrasted to the current solution.*

Free form visualization section could go here


# Alt
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