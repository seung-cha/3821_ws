Source Code for COMP3821

# Map Exapnder

```py
def ExpandMap(self, robotRadius, factor:float, map:Map)
```

Given a map, it will enlarge each occupied cell by the following formula:

floor(robotRadius * factor).

Where:

* `robotRadius`: Number of cells to enlarge (radius).
* `factor`: Factor.


It basically replaces each occupied cell with a square occupied cell whose sidelength is floor(robotRadius * factor) * 2.







