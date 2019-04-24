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


# input- numpy.ndarray of xyz_cones.values (from cone_finder func)
# output- pandas dataframe of xyz
def to_pandas_df(bytestr):
    decodedStr = np.frombuffer(bytestr)

    decodedStr = np.reshape(decodedStr, (int(len(decodedStr)/3), 3))
    # print(decodedStr[:, 0:2])
    # df = pd.DataFrame(decodedStr)
    return decodedStr[:, 0:2]


class Server:
    def __init__(self):
        UDP_IP = "0.0.0.0"
        UDP_PORT = 4000
        self.addr = (UDP_IP, UDP_PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.addr)
        print("UDP server up and listening")


    def receive_packet(self):
        data, addr = self.sock.recvfrom(60000)   # buffer size - 2000
        clientMsg = to_pandas_df(data)
        return clientMsg


class Client:
    def __init__(self):
        UDP_IP = "192.168.0.101"
        UDP_PORT = 4000
        self.addr = (UDP_IP, UDP_PORT)
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    def send_packet(self, data):
        self.sock.sendto(data, self.addr)


class PDF:
    def __init__(self, fig):
        x = np.linspace(-15, 15, 200)  # each block will be 20cm wide
        y = np.linspace(-15, 15, 200)  # each block will be 20cm wide
        self.X, self.Y = np.meshgrid(x, y)
        sigma = np.array([0.5, 0.5])
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

# need to choose the specific activation according to client/server functionality
def main():
    print("Running Server script")
    plt.ion()
    fig = plt.figure(1)
    mng = plt.get_current_fig_manager()
    mng.full_screen_toggle()

    server = Server()
    PDF_map = PDF(fig)
    Z = 0
    while True:
        cone_vec = server.receive_packet()
        Z = PDF_map.calc_pdf(cone_vec, Z)
        PDF_map.plot_pdf_map(fig, Z)


if __name__ == '__main__':
    main()