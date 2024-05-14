x_counts = {}
y_counts = {}

with open('/home/livan/Desktop/map_coordinates.txt', 'r') as file:
    for line in file:
        x, y, value = line.strip().split(',')
        x = int(float(x.split(': ')[1]))
        y = int(float(y.split(': ')[1]))
        if x not in x_counts:
            x_counts[x] = 0
        if y not in y_counts:
            y_counts[y] = 0
        x_counts[x] += 1
        y_counts[y] += 1

with open('/home/livan/Desktop/frequency.txt', 'w') as file:
    file.write('Frequency of x values:\n')
    for x, count in x_counts.items():
        file.write('x={}: {}\n'.format(x, count))

    file.write('Frequency of y values:\n')
    for y, count in y_counts.items():
        file.write('y={}: {}\n'.format(y, count))

