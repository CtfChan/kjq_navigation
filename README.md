# kjq_navigation

## Overview

Our awesome autonomous rover with a 2D lidar. For system diagram and architecture details go here: https://docs.google.com/document/d/1UoM3MzzAnzNUYfwepUr3FkmhO7stVCdSh5XprO91oN4/edit?usp=sharing. 



# Unit Tests
```
catkin build --make-args tests
rosrun kjq_navigation kjq_navigation-test 
```



## Dependencies


``` 
sudo apt-get install ros-kinetic-grid-map
sudo apt-get install ros-kinetic-teleop-twist-keyboard

```
More details here:
https://personalrobotics.cs.washington.edu/software/unit-testing/


## Nodes

### random_drive_node

Uses lidar to randomly drive around the map while avoiding obstacles.

### global_planner_node

Maintains FSM of robot. Generates globally optimal path and provides it to the local_planner.

## Task List
- [x] Create package template
- [x] Implement random drive
- [x] Generate tf file for laser->base_link
- [ ] Generate map with Gmapping
- [ ] Try to use AMCL to solve kidnapped robot problem
- [ ] Enable Gtest for our project in writing A*
- [ ] Implement global planner and test using blank grid map 
- [ ] Implement local planner
- [ ] Do integration testing of AMCL with planners
- [ ] Implement block detector







