Source Code for COMP3821

# Artificial Potential Field

Suppose the robot and obstacles are positively charged and the goal negative.

The robot is attracted to the goal and repulsed by obstacles. The resulting behaviour is robot navigating to the goal while avoding any obstacle.

Each cell has potential which is the sum of attractive potential (goal) and repulsive potential (obstacles). The direction at which the robot moves is defined by the gradient of each cell.

Path is computed numerically using Gradient Descendent Method.