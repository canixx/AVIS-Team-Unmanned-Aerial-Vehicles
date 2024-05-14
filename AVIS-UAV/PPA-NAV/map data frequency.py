import collections

with open('/home/livan/Desktop/map_matrix.txt', 'r') as file:
    data = [float(val) for line in file for val in line.strip().split()]

value_counts = collections.Counter(data)

with open('/home/livan/Desktop/map_value_counts.txt', 'w') as file:
    for value, count in value_counts.items():
        file.write("Value: {:.6f}, Count: {}\n".format(value, count))

