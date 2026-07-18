import numpy as np

arr1 = np.array([
    [0, 1, 1, 0, 0],
    [1, 0, 0, 0, 0],
    [1, 1, 0, 1, 1],
    [0, 0, 0, 0, 0],
    [1, 1, 1, 1, 0]
])

arr2 = np.array([
    1, 1, 1, 1, 1
])

print(arr1@arr2.T)
