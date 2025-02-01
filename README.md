# Autonomous-Driving-with-Turtlebot3

## Final project :- TurtleBot Path-finding and Automatic Parking

# Code structure

## 1. Wall follower & finding goal 
 The [wall follower](https://github.com/boradj/Turtlebot3/blob/main/Lab4_final/Code/test_spot.py) class will navigate through its surroundings using a wall follower algorithm. The process consists of the following steps: 
 - First robot will save the starting position coordinate and start moving towards closest wall. 
 - The robot will locate the closest wall and save the cordinates. 
 - It will then follow the wall while avoiding any obstacles.
 - Once the robot has completed a full lap of the environment, it will come to a halt and store the all possible parking spot.

## 2.Mapping 
 - The [mapping](https://github.com/boradj/Turtlebot3/blob/main/Lab4_final/Code/mapping.py) class will operate simultaneously with 
   the wall follower function, using LIDAR values to gather information about the environment and its positions. 
 - It will store data about all detected obstacles in the form of coordinates and keep a record of this information in a file called "ranges.txt" when robot will reach    coordinates of nearest position.

## 3.Trajectory planning
 - The [path planning](https://github.com/boradj/Turtlebot3/blob/main/Lab4_final/Code/pathplanningandpathfollow.py) class, using the [A* algorithm](https://github.com/boradj/Turtlebot3/blob/main/Lab4_final/Code/Astar.py), will take three inputs: the map, the current position of the robot, and the desired destination.  
 - It will then generate all potential paths, compare them, and choose the shortest path to reach the goal.

## 4. Path follower
 - The [path planning](https://github.com/boradj/Turtlebot3/blob/main/Lab4_final/Code/pathplanningandpathfollow.py) class will receive the points of the shortest path generated by the Astar planning class.
 - It utilize a PI controller to guide the robot towards the destination along this path.

## Project implementation map
<img src="https://github.com/boradj/Turtlebot3/blob/main/Lab4_final/Images/Real%20Enviroment%20Map%202.jpeg" width="350" height="350"> <img src="https://github.com/boradj/Turtlebot3/blob/main/Lab4_final/Images/Real%20enviroment%20Map.jpeg" width="350" height="350">

##  Map of Real Enviroment 
  <img src="https://github.com/boradj/Turtlebot3/blob/main/Lab4_final/Images/Map%20generated%20by%20Mapping%20algorithm.jpeg" width="400" height="410">

##  Map of A* Star Algorithm
 Task 1:
 Path Planning towards starting position   and Path Planning towards parking spot                                         
 
 <img src="https://github.com/boradj/Turtlebot3/blob/main/Lab4_final/Images/goingtoinialpositionsfirstttask.png" width="350" height="350"> <img src="https://github.com/boradj/Turtlebot3/blob/main/Lab4_final/Images/finalparkingpathforfirsttask.png" width="350" height="350">
  
 Task 2:
 Path Planning towards starting position  and Path Planning towards parking spot
 
<img src="https://github.com/boradj/Turtlebot3/blob/main/Lab4_final/Images/goingtoinitialpositionsecondtask.png" width="400" height="410"> <img src="https://github.com/boradj/Turtlebot3/blob/main/Lab4_final/Images/finalparkingpathforsecondtask.png" width="400" height="410">
