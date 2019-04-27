#!/usr/bin/python

import socket
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
import matplotlib.cm as cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import time
from mpl_toolkits.mplot3d import axes3d, Axes3D
from sklearn.cluster import KMeans
from utilities import plot_telemetry


'''
Abstract:   bytes_to_numpyArray func is used to change input bytes array to np.array.
Parameters: bytes string of xy_cones.values (from cone_finder func) and the last line of data is x_car, y_car.
Returns:    xy_cones- np.ndarray of xy points of all cones in world system
            xy_car- np.ndarray of xy points of car's position according to LAT/LONG of the IMU
            ypr - Yaw, Pitch & Roll from IMU
'''
def bytes_to_numpyArray(bytestr):
    decodedStr = np.frombuffer(bytestr)
    decodedStr = np.reshape(decodedStr, (int(len(decodedStr)/2), 2))
    xy_car = decodedStr[-3]
    ypr = [decodedStr[-2][0],decodedStr[-2][1], decodedStr[-1][0]]
    xy_cones = decodedStr[:-3, :]
    return xy_cones, xy_car, ypr


class Server:
    def __init__(self):
        UDP_IP = "0.0.0.0"
        UDP_PORT = 4000
        self.addr = (UDP_IP, UDP_PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.addr)
        print("UDP server up and listening")


    def receive_packet(self):
        data, addr = self.sock.recvfrom(60000)   # buffer size
        xy_cones, xy_car, ypr = bytes_to_numpyArray(data)
        return [xy_cones, xy_car, ypr]


class Client:
    def __init__(self):
        UDP_IP = "192.168.0.103"  # IMPORTANT NOTE - please do check your router IP and change it accordingly
        UDP_PORT = 4000
        self.addr = (UDP_IP, UDP_PORT)
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    def send_packet(self, data):
        self.sock.sendto(data, self.addr)


'''
Abstract:   kmeans_cluster class responsible for K-means algorithm clustering.
            It's important to notice that in "calc_k_centers" method the user can (and maybe should) change the number
            of clusters needed for his project (K), and the varible "frame_count".
            These variables depends on car velocity and on expected number of cones in each frame
'''
class kmeans_cluster:
    def __init__(self):
        self.temp_dataFrame = pd.DataFrame()
        self.cone_img_list = []
        self.car_position = []
        self.IDX = 0
        self.images_path = 'C:/Users/Administrator/Desktop/project-Aviv_Avinoam'
        self.k = 4 # number of centers to calc for k-means algorithm
        self.frame_time = time.time()
        self.frame_count = 50

    def calc_k_centers(self, data):
        xyCones_world = pd.DataFrame(data[0])
        x_car,y_car = data[1]
        ypr = data[2]
        self.temp_dataFrame = self.temp_dataFrame.append(xyCones_world)
        self.car_position.append([x_car,y_car])
        self.car_position = np.unique(self.car_position, axis=0).tolist()
        if self.IDX % self.frame_count == 0:
            cone_vec = np.array(self.temp_dataFrame.values)[:, 0:2]
            kmeans = KMeans(n_clusters=self.k, random_state=0).fit(cone_vec)
            self.cone_img_list.append(kmeans.cluster_centers_)
            plot_telemetry(self.cone_img_list, self.car_position, ypr, self.frame_time)
            self.temp_dataFrame = pd.DataFrame()
        self.IDX += 1

    def plot_map(self):
        cone_img = np.concatenate(self.cone_img_list, axis=0)
        car_route_img = np.reshape(self.car_position, (len(self.car_position),2))
        plt.scatter(car_route_img[:,0], car_route_img[:,1], s=50, c='b', marker='o', label='Car\'s Route')
        plt.scatter(cone_img[:,0],cone_img[:,1], s=100, c='r', marker='^', label='Cones Positions')
        curr_time = time.time() - self.frame_time
        plt.legend(loc='best')
        plt.title('Cones Clustering Map - K-means Algorithm \n Frame timeStamp-{} sec'.format(round(curr_time, 3)),
                  {'fontsize': 20}, loc='center')
        plt.xlabel('x-axis (right / left) [m]')
        plt.ylabel('y-axis (forward / backward) [m]')
        plt.draw()
        plt.pause(0.1)
        plt.clf()

    def save_fig(self, plt):
        imageFile = self.images_path + "/image_{}.png".format(self.IDX)
        plt.savefig(imageFile)


'''
Abstract:   PDF class responsible for gaussian probability densitiy mapping method.
            It's important to notice that in "calc_pdf" method the user can (and maybe should) change the sigma value,
            or the amplitude value, or the grid size.
'''
class PDF:
    def __init__(self, fig):
        x = np.linspace(-15, 15, 200)  # each block will be 20cm wide
        y = np.linspace(-15, 15, 200)  # each block will be 20cm wide
        self.X, self.Y = np.meshgrid(x, y)
        sigma = np.array([0.3, 0.1])
        self.covariance = np.diag(sigma ** 2)
        self.amp = 4 / (sigma[0] * sigma[1] * np.pi)
        self.frame_time = time.time()
        self.images_path = 'C:/Users/Administrator/Desktop/project-Aviv_Avinoam'
        self.IDX = 0
        self.xy = np.column_stack([self.X.flat, self.Y.flat])

    def calc_pdf(self, cone_vec, old_Z):
        Z = old_Z + np.sum([self.amp * multivariate_normal.pdf(self.xy, mean=np.array(cone_vec[i]), cov=self.covariance).reshape(self.X.shape)
                            for i in range(len(cone_vec))], axis=0)
        return Z
        # print(self.Z)

    def save_fig(self, plt):
        imageFile = self.images_path + "/image_{}.png".format(self.IDX)
        self.IDX += 1
        plt.savefig(imageFile)

    def plot_pdf_map(self, fig, Z):
        ax = fig.add_subplot(111, projection='3d')
        plt.gca().invert_xaxis()
        plt.gca().invert_yaxis()
        surf = ax.plot_surface(self.X, self.Y, Z, cmap=cm.coolwarm, linewidth=0.1)
        ax.view_init(90, 0)
        curr_time = time.time() - self.frame_time
        plt.title('Cones Probability Map \n Frame timeStamp-{} sec'.format(round(curr_time, 3)), {'fontsize': 20},
              loc='center')
        plt.xlabel('x-axis (right / left)')
        plt.ylabel('y-axis (forward / backward)')
        # Customize the z-axis.
        ax.zaxis.set_major_locator(LinearLocator(10))
        ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
        fig.colorbar(surf, shrink=0.9, aspect=10)
        # Add a color bar which maps values to colors.
        ax.set_aspect('equal')
        plt.draw()
        # self.save_fig(plt)
        plt.pause(0.0001)
        plt.clf()


'''
Abstract:   In this Main function the user must choose the specific activation according to client/server functionality
            Server mode (work station) will activate this main function alone.
            Client mode (car) will activate "Main_t" script alone
            Please note that we choose to use k-means algorithm as clustering method, so the server is activating it
            after each data packet (cones at global system) it gets.
'''
def main():
    print("Running Server script")
    plt.ion()
    fig = plt.figure(1)
    mng = plt.get_current_fig_manager()
    mng.full_screen_toggle()

    server = Server()
    k_cluster = kmeans_cluster()

    while True:
        data = server.receive_packet()
        k_cluster.calc_k_centers(data)


if __name__ == '__main__':
    main()