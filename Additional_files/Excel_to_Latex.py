from math import *
# Program extracting first column
import xlrd


# Give the location of the file
loc = ("Excel_with_values.xlsx")

# To open Workbook
wb = xlrd.open_workbook(loc)
sheet = wb.sheet_by_index(0)

indent = '    '
latex_code = '\\begin{bmatrix} \n'  # Initializing the latex_code variable
for j in range(sheet.nrows):  # Loops through the rows
    latex_code += indent  # Adds an indent
    for i in range(sheet.ncols):  # Loops through the columns
        latex_code += str(round(sheet.cell_value(j, i), 4)) + ' & '  # Adds the value and separates with the & sign
    latex_code += ' \\\ \n'  # Adds a new row
latex_code += '\\end{bmatrix} \n'  # Close the matrix statement
print(latex_code)




