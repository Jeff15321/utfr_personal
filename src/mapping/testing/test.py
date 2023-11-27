import matplotlib.pyplot as plt

data = '''
[build_graph-2] 0 4.5928 5.15169
[build_graph-2] 1 6.8748 1.74585
[build_graph-2] 2 7.34886 -6.3129
[build_graph-2] 3 9.00266 -3.89072
[build_graph-2] 4 6.81229 -9.21262
[build_graph-2] 5 11.521 -2.18945
[build_graph-2] 6 7.37321 -12.1551
[build_graph-2] 7 14.3944 -1.62831
[build_graph-2] 8 4.61331 -5.19568
[build_graph-2] 9 6.89734 -1.74442
[build_graph-2] 10 7.37249 6.33999
[build_graph-2] 11 8.99684 3.87845
[build_graph-2] 12 6.79249 9.27012
[build_graph-2] 13 11.4718 2.22869
[build_graph-2] 14 7.3664 12.139
[build_graph-2] 15 14.3931 1.62028
[build_graph-2] 16 12.8959 -1.80861
[build_graph-2] 17 12.9304 1.76655
[build_graph-2] 18 3.40249 1.64648
[build_graph-2] 19 3.42086 -1.64397
[build_graph-2] 20 5.37174 1.6317
[build_graph-2] 21 5.397 -1.64068
[build_graph-2] 22 15.8644 1.76263
[build_graph-2] 23 15.8795 -1.75883
[build_graph-2] 24 17.2808 -2.22809
[build_graph-2] 25 17.317 2.20193
[build_graph-2] 26 19.723 -3.90642
[build_graph-2] 27 19.7793 3.87691
[build_graph-2] 28 21.8691 -1.73808
[build_graph-2] 29 21.9052 1.73473
[build_graph-2] 30 21.4059 -6.35249
[build_graph-2] 31 21.4188 6.34208
[build_graph-2] 32 23.3913 1.66226
[build_graph-2] 33 23.3817 -1.62915
[build_graph-2] 34 24.1469 5.1904
[build_graph-2] 35 24.1839 -5.19654
[build_graph-2] 36 21.9802 -9.23827
[build_graph-2] 37 22.0306 9.20354
[build_graph-2] 38 25.3832 -1.64278
[build_graph-2] 39 25.3628 1.63545
[build_graph-2] 40 27.3721 -1.61086
[build_graph-2] 41 27.3677 1.60523
[build_graph-2] 42 21.4107 -12.1607
[build_graph-2] 43 21.401 12.1723
[build_graph-2] 44 24.9789 9.24919
[build_graph-2] 45 24.9848 -9.25775
[build_graph-2] 46 29.4043 -1.64337
[build_graph-2] 47 29.3832 1.63852
[build_graph-2] 48 31.3726 -1.64396
[build_graph-2] 49 31.3741 1.65351
'''

lines = data.split('\n')

columns = [line.split() for line in lines]

x = [float(column[2]) for column in columns if column]
y = [float(column[3]) for column in columns if column]

plt.plot(x, y, 'o')
plt.show()
