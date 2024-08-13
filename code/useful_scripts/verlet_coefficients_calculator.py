import numpy as np


num_points = 7
num_columns = num_points + 1

RHS = np.zeros(num_columns)
RHS[2] = 1

taylor_coef_matrix = np.zeros((num_columns, num_columns))

bases = [j for j in range(-num_points+1, 2)]

for i in range(num_columns):
    row = []
    for base in bases:
        row.append(base**i)
    taylor_coef_matrix[i] = row

x = np.linalg.solve(taylor_coef_matrix, RHS)


multiplier = 1 / x[-1]
x = x[0:-1]
x = -1*x
x = np.append(x, [0.5])
x = x * multiplier

print(x)
