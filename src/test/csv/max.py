import csv

def get_max_value(csv_file, column):
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        values = [float(row[column]) for row in reader]
        max_value = max(values)
        return max_value

# Example usage
csv_file = '/home/naga/klab_ws/src/test/csv/l1_-1.3_-5_3.csv'
column_name = 'output_pos'
max_value = get_max_value(csv_file, column_name)
print(f"The maximum value in column '{column_name}' is: {max_value}")
