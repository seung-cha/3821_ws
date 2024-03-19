Source Code for COMP3821

# Weighted Map

The heuristic function is modified as follows:
```
C(x) = g(x) + h(x) + c*w(x)
```
Where `C(x)` represents the cost of cell `x`, `g(x), h(x)` respectively represents the calculated distance and heuristic (i.e manhattan distance).

`c` is coefficient and `w(x)` is the weight of cell `x`.

Occupied cells and the cells around them are given a weight (0 ~ 1]. Higher values significantly disencourages the robot from taking that cell.

The coefficient `c` is to be adjusted. Extremely low value makes the robot ignore the weight (same as vanilla A*) and extremely high value makes the robot not consider the cell at all (same as Expanded Map)

As opposed to expanded map, robot is not forced to ignore cells adjacent to occupied cells. The robot is instead disencouraged to use them.


It may not work; it's just an experimental map inspired by Artificial Potential Field and Guideline Based A*.







