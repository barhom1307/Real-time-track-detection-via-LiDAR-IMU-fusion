#!/usr/bin/python

import socket
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import text
import time


# input- numpy.ndarray of xyzt_cones.values (from cone_finder func)
# output- pandas dataframe of xyzt
def to_pandas_df(bytestr):
    decodedStr = np.frombuffer(bytestr)
    decodedStr = np.reshape(decodedStr, (int(len(decodedStr)/4), 4))
    df = pd.DataFrame(decodedStr)
    return df


class Server:
    def __init__(self):
        UDP_IP = "0.0.0.0"
        UDP_PORT = 4000
        self.addr = (UDP_IP, UDP_PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.frame_time = time.time()

    def receive_packet(self, fig):
        self.sock.bind(self.addr)
        print("UDP server up and listening")
        while True:
            data, addr = self.sock.recvfrom(60000)   # buffer size - 2000
            clientMsg = to_pandas_df(data)
            clientIP = "Client IP Address:{}".format(addr)
            # print(clientMsg)
            plt.scatter(clientMsg.iloc[:, 0], clientMsg.iloc[:, 1], c='r', s=50, marker='^', alpha=0.8)
            timestamp = (time.time()-self.frame_time)
            print(timestamp)
            title = 'LIVE LIDAR PLOT- ' + str(timestamp) + ' sec'
            plt.title(title, fontsize=14)
            plt.xlabel('X [m]')
            plt.ylabel('Y [m]')
            plt.xlim((-3, 3))
            plt.ylim((0, 10))
            plt.draw()
            plt.pause(0.0001)
            plt.clf()


class Client:
    def __init__(self):
        UDP_IP = "192.168.0.101"
        UDP_PORT = 4000
        self.addr = (UDP_IP, UDP_PORT)
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    def send_packet(self, data):
        self.sock.sendto(data, self.addr)


# need to choose the specific activation according to client/server functionality
def main():
    print("Running Server script")
    # client = Client()
    # while True:
    #     client.send_packet("Hello World!")
    plt.ion()
    mng = plt.get_current_fig_manager()
    mng.full_screen_toggle()
    fig = plt.figure(1)
    server = Server()
    while True:
        server.receive_packet(fig)


if __name__ == '__main__':
    main()
