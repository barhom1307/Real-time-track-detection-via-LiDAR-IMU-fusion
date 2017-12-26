from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import os

read_path = '/home/avinoam/Documents/text_Lidar'  # please insert the desired path on your computer
file_name = 'data_file_5.txt'  # please insert the desired file name
save_name = os.path.join(read_path, file_name)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

xs = []
ys = []
zs = []

def animate(i):
    graph_data = open(save_name, "r")
    for line in graph_data:
            x = float(line.split()[0])
            y = float(line.split()[1])
            z = float(line.split()[2])
            xs.append(x)
            ys.append(y)
            zs.append(z)
    ax.scatter(xs, ys, zs, c='g', marker='o', alpha=0.7, s=20)


ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.title('Lidar PointCloud in realtime event')
plt.show()