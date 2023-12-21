import matplotlib.pyplot as plt

with open('pose.txt', 'r') as f:
    data = f.read()

lines = data.split('\n')

columns = [line.split() for line in lines]

x = [float(column[1]) for column in columns if column]
y = [float(column[2]) for column in columns if column]


plt.plot(x, y, 'o')
plt.show()
