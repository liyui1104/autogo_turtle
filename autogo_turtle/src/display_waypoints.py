import matplotlib.pyplot as plt
import pandas as pd

file_names = ['A.csv', 'u.csv', 't.csv', 'o1.csv', 'G.csv', 'o2.csv']

# Draw
plt.figure(figsize=(6, 6))
plt.xlim(0, 11)
plt.ylim(0, 11)
plt.gca().set_aspect('equal', adjustable='box')

for file_name in file_names:
    # Read data from .csv files
    df = pd.read_csv(file_name, header=None, names=['x', 'y'])
    
    # Draw points
    plt.scatter(df['x'], df['y'], label=file_name[:-4])

    # Connect points with straight lines
    for i in range(1, len(df)):
        plt.plot([df['x'][i-1], df['x'][i]], [df['y'][i-1], df['y'][i]])

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Coordinate Points Plot')
plt.grid(True)
plt.legend()
plt.show()
