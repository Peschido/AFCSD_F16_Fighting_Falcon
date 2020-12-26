from math import *
# Program extracting first column
import xlrd
import numpy as np


# Control Panel------------------------------------------------------------------
# Add the cross out function to the output
add_cross_out = False
indent = '    '
cross_out_row = 0
cross_out_column = 0
# Do we want to reduce the rows?
reduce_amount = 1
# Do we want to change rows and columns around
frankenstein_matrix = True

original_seq = ['theta', 'v', 'alpha', 'q', 'delta_t', 'delta_e']
new_seq = ['v', 'alpha', 'theta', 'q', 'delta_e', 'delta_t']

# Code---------------------------------------------------------------------------
# Give the location of the file
loc = ("Excel_with_values.xlsx")

# To open Workbook
wb = xlrd.open_workbook(loc)
sheet = wb.sheet_by_index(0)
rows = sheet.nrows
columns = sheet.ncols
j = reduce_amount
i = reduce_amount
array_original = np.zeros((rows - reduce_amount, columns - reduce_amount))  # Create array
array_rows_changed = np.zeros((rows - reduce_amount, columns - reduce_amount))  # Create array
array_new = np.zeros((rows - reduce_amount, columns - reduce_amount))  # Create array

if frankenstein_matrix:
    reduce_matrix = True
    # Fill array with the correct values
    for j in range(rows - reduce_amount):
        for i in range(columns - reduce_amount):
            array_original[j, i] = round(sheet.cell_value(j+reduce_amount, i+reduce_amount), 4)
    # Change the rows around
    for i in range(len(original_seq)):
        array_rows_changed[new_seq.index(original_seq[i]), :] = array_original[i, :]
    # Change the columns around
    for i in range(len(original_seq)):
        array_new[:, new_seq.index(original_seq[i])] = array_rows_changed[:, i]

print('The amount of rows is: ', rows)
print('The amount of rows is: ', columns)

latex_code = '\\begin{bmatrix} \n'  # Initializing the latex_code variable
for j in range(rows - reduce_amount):  # Loops through the rows
    latex_code += indent  # Adds an indent
    for i in range(columns - reduce_amount):  # Loops through the columns
        if frankenstein_matrix:
            a = str(round(array_new[j, i], 4))
        else:
            a = str(round(sheet.cell_value(j + reduce_amount, i + reduce_amount), 4))
        if i == cross_out_row and add_cross_out or j == cross_out_column and add_cross_out:
            latex_code += '$\sout{' + a + '}$' + ' & '
        else:
            latex_code += a + ' & '  # Adds the value and separates with the & sign
    latex_code = latex_code[:-2]  # Removes the & before \\
    latex_code += '\\\ \n'  # Adds a new row
latex_code += '\\end{bmatrix} \n'  # Close the matrix statement
print(latex_code)







