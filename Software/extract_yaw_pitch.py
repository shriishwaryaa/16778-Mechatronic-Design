import csv

def find_row_by_first_column(filename, search_value):
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            if row[0] == search_value:
                return row
    return None

# Example usage
filename = 'output_joint_data.csv'
search_value = '6'
result_row = find_row_by_first_column(filename, search_value)

if result_row:
    print("Found row:", result_row)
else:
    print("No row found with that value.")
