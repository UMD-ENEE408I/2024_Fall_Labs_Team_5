import numpy as np
from numpy.linalg import eig

np_array = np.array([1, 2, 3, 4])
print("3. array: \n", np_array)

ones_array = np.ones((3, 4))
zeros_array = np.zeros((4,3))
print("4. ones array: \n", ones_array)
print(" zeros array: \n", zeros_array)

A = np.array([[1, 2, 3], [4, 5, 6]])
B = np.array([[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12]])

C = np.dot(A, B)

print("matrix a: \n", A)
print("* matrix b: \n", B)
print( "= \n", C)

mat_for_eig = np.array([[3, 1], [1, 2]])
print("for matrix: \n", mat_for_eig)

evals, evecs = eig(mat_for_eig)
print(" eigenvalues are: ", evals)
print(" eigenvectors are: ", evecs)


