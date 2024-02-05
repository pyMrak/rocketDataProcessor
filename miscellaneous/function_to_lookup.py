from matplotlib import pyplot as plt
from numpy import linspace, sin, array, append
from tqdm import tqdm

def interpolate_2d_point(point1, point2, x):
    """
    Perform linear interpolation between two 2D points.

    Parameters:
        point1 (tuple): The first 2D point (x1, y1).
        point2 (tuple): The second 2D point (x2, y2).
        x (float): The x coordinate of interpolated point

    Returns:
        interpolated_y: The interpolated y coordinate of 2D point.
    """
    x1, y1 = point1
    x2, y2 = point2

    interpolated_y = y1 + (y2 - y1) * (x - x1) / (x2 - x1)

    return interpolated_y

def generate_lookup_table_pair(func, x_range, max_diff_percentage):

    x_min, x_max = x_range

    _x = linspace(x_min, x_max, 10001)
    _y = func(_x)
    y_min = min(_y)
    y_max = max(_y)
    x = _x[:1]

    max_diff = (y_max - y_min) * max_diff_percentage / 100
    i = 0
    for _x_item in tqdm(_x[1:]):
        repeat = True
        while repeat:
            repeat = False
            __x = linspace(x[-1], _x_item, 1000)
            __y = func(__x)
            point1 = (__x[0], __y[0])
            point2 = (__x[-1], __y[-1])
            for __x_item, __y_item in zip(__x, __y):
                __y_interp = interpolate_2d_point(point1, point2, __x_item)
                if abs(__y_interp - __y_item) > max_diff:
                    repeat = True
                    x = append(x, _x[i])  # should be _x[i - 1] but we are iterating _x[1:]
                    break
        i += 1
    if _x[-1] != x[-1]:
        x = append(x, _x[-1])

    x = array(x)
    y = func(x)

    return x, y





# Example usage:
import math


# Define an arbitrary function
def arbitrary_func(x):
    return sin(x)


x = linspace(0, 2 * math.pi, 100)
y = arbitrary_func(x)

# Generate lookup table pair
table_x, table_y = generate_lookup_table_pair(arbitrary_func, (0, 2 * math.pi), 0.05)
plt.plot(x, y)
plt.scatter(table_x, table_y)
print(len(table_x))
print("[", end="")
for x in table_x:
    print(x, end=", ")
print("]")

print("[", end="")
for y in table_y:
    print(y, end=", ")
print("]")


plt.grid()
plt.show()