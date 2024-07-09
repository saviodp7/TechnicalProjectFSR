from ulab import numpy as np
from math import sin, cos

def add_vectors(A, B):
    result = np.zeros(len(A))
    for i in range(len(A)):
        result[i] = A[i] + B[i]
    return result

def multiply_scalar_matrix(scalar, matrix):
    result = np.zeros(matrix.shape)
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            result[i][j] = scalar * matrix[i][j]
    return result

def outer_product(vector1, vector2=None):
    if vector2 is None:
        vector2 = vector1
    n = len(vector1)
    m = len(vector2)
    result = np.zeros((n,m))
    for i in range(n):
        for j in range(m):
            result[i][j] = vector1[i] * vector2[j]
    return result

def deep_copy(matrix):
    rows,cols = matrix.shape
    copied_matrix = np.zeros((rows,cols))
    for i in range(rows):
        for j in range(cols):
            copied_matrix[i][j] = matrix[i][j]
    return copied_matrix

def deep_copy_vect(array):
    leng=len(array)
    copied_array = np.zeros(leng)
    for i in range(leng):
        copied_array[i] = array[i]
    return copied_array

def threshold_matrix(matrix, threshold):
    rows,cols = matrix.shape
    
    for i in range(rows):
        for j in range(cols):
            if matrix[i][j] < threshold:
                matrix[i][j] = 0
                
    return matrix

def block_diag(*matrices):
    # Determina la dimensione totale della matrice risultante
    total_rows = sum(len(m) for m in matrices)
    total_cols = sum(len(m[0]) for m in matrices)

    # Inizializza la matrice risultante con zeri
    result = [[0 for _ in range(total_cols)] for _ in range(total_rows)]

    # Copia ogni matrice nella posizione corretta
    current_row = 0
    current_col = 0
    for matrix in matrices:
        rows = len(matrix)
        cols = len(matrix[0])
        for i in range(rows):
            for j in range(cols):
                result[current_row + i][current_col + j] = matrix[i][j]
        current_row += rows
        current_col += cols

    return result

