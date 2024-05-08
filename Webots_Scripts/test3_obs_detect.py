import numpy as np

# Create 5 sample arrays (you can replace these with your actual data)
array1 = np.array([[1, 2, 3], [4, 5, 6]])
array2 = np.array([[7, 8, 9], [10, 11, 12]])
array3 = np.array([[13, 14, 15], [16, 17, 18]])
array4 = np.array([[19, 20, 21], [22, 23, 24]])
array5 = np.array([[25, 26, 27], [28, 29, 30]])

# Stack the arrays using np.dstack
stacked_array = np.dstack((array1, array2, array3, array4, array5))

# Optionally, encapsulate the stacked array in a list
# list_of_objects = [stacked_array]

print(stacked_array)
