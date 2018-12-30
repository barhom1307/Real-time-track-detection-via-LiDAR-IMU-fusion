#!/usr/bin/python

from vnpy import *
import pandas as pd
pd.set_option('float_format', '{:f}'.format)
import numpy as np
from scipy.stats import linregress
import pickle
import time
from multiprocessing import Process, Queue
from signal import signal, SIGTERM
import os
import matplotlib.pyplot as plt
from matplotlib import style
import matplotlib.animation as animation
import scipy.interpolate as interp
from mpl_toolkits.mplot3d import axes3d

import RotMatrixes
import utilities
style.use('fivethirtyeight')

NUM_OF_PACKETS_LIDAR = 754  # number of lidar packets per 1 sec
NUM_OF_PACKETS_IMU = 20     # number of IMU packets per 1 sec
COLUMNS_IMU = 7             # number of IMU data columns
FLAG = True


def handler(signum, frame):
    global FLAG
    FLAG = False


class vn200_online_feeder:
    def __init__(self, imu_pkl):
        self.IMU_PKL = imu_pkl
        self.run()


    def receiver(self, queue):
        sensor = VnSensor()
        print('Establishing Connection with IMU...')
        sensor.connect('/dev/ttyUSB0', 115200)  # Default = 115200, fastest 921600
        sensor.change_baudrate(921600)
        print('Connection Established, ' + 'Baudrate is: ' + str(sensor.read_serial_baudrate()))
        signal(SIGTERM, handler)
        self.imu_packets = []
        while FLAG:
            lla = sensor.read_ins_solution_lla()
            lla_data = [
                lla.yawPitchRoll.x,  # Yaw
                lla.yawPitchRoll.y,  # Pitch
                lla.yawPitchRoll.z,  # Roll
                lla.position.x,      # Lat
                lla.position.y,      # Long
                lla.position.z,      # Alt
                time.time()
            ]
            queue.put(lla_data)
            self.imu_packets.append(lla_data)
        sensor.change_baudrate(115200)
        with open(self.IMU_PKL, 'wb') as imu_f:
            pickle.dump(self.imu_packets, imu_f)
        os._exit(0)

    def run(self):
        self.queue = Queue()
        self.proc = Process(target=self.receiver, args=(self.queue,),name='IMU_proc')
        self.proc.start()

    def get_packet(self):
        return self.queue.get()

    def close_imu(self):

        print('Terminating IMU Connection...')
        self.proc.terminate()


class vn200_offline_feeder:
    def __init__(self, imu_pkl):
        self.IMU_PKL = imu_pkl  # /...path.../../imu_rec_test.pkl
        imu_array = []
        print("Connecting to offline IMU feeder...")
        with open(self.IMU_PKL, 'rb') as imu_f:
            while True:
                try:
                    imu_packet = pickle.load(imu_f)
                    imu_array = np.append(imu_array, imu_packet)
                except EOFError:
                    break
        imu_array = np.reshape(imu_array, (int(len(imu_array)/COLUMNS_IMU), COLUMNS_IMU))
        self.imu_list = imu_array.tolist()
        #print(self.imu_list)

    def get_packet(self):
        return self.imu_list.pop(0)


fig = plt.figure()
ax1 = fig.add_subplot(111, projection='3d')


class vn200_decoder:

    def __init__(self, packet_feeder):
        self.packet_feeder = packet_feeder

    def get_imu_frame(self, lidar_frame_time):
        imu_t1 = self.packet_feeder.get_packet()
        while abs(imu_t1[6] - lidar_frame_time[0]) > 0.01:
            imu_t1 = self.packet_feeder.get_packet()
        imu_t2 = self.packet_feeder.get_packet()
        return [imu_t1, imu_t2]

    # [1, 2, 3, 4, 5, 6, 0], [2, 3, 4, 5, 6, 7, 1], [-1, 1, 100])
    def imu_interpolate(self, imu_data1, imu_data2, lidar_frame_time):  # 1x7, 1x7, 1xlen(lidar_time)) || LIDAR_XYZT[3]
        self.interp_imu_frame = pd.DataFrame(np.zeros((len(lidar_frame_time), 7)))
        imu_data = pd.DataFrame([imu_data1[0:6], imu_data2[0:6]])  # 2x6, without time values
        time_vec = [imu_data1[6], imu_data2[6]]

        imu_data_slope = imu_data.apply(lambda y: linregress(time_vec, y).slope, axis=0)
        # .apply passes each of the columns of imu_data (shape: [2x1]) to y.

        imu_data_intercept = imu_data.apply(lambda y: linregress(time_vec, y).intercept, axis=0)
        # .apply passes each of the columns of imu_data (shape: [2x1]) to y.

        linear_df = pd.DataFrame([imu_data_slope, imu_data_intercept])
        for i, t in enumerate(lidar_frame_time):
            self.interp_imu_frame.iloc[i, :] = linear_df.apply(lambda n: n[0] * t + n[1], axis=0)
        self.interp_imu_frame[6] = lidar_frame_time

    def rotation_matrix(self, ypr):
        yaw = ypr[0]
        pitch = ypr[1]
        roll = ypr[2]

        R_Mat_Yaw = np.mat([[np.cos(yaw * (np.pi / 180)), np.sin(yaw * (np.pi / 180)), 0],
                            [-np.sin(yaw * (np.pi / 180)), np.cos(yaw * (np.pi / 180)), 0],
                            [0, 0, 1]])

        R_Mat_Pitch = np.mat([[np.cos(pitch * (np.pi / 180)), 0, -np.sin(pitch * (np.pi / 180))],
                              [0, 1, 0],
                              [np.sin(pitch * (np.pi / 180)), 0, np.cos(pitch * (np.pi / 180))]])

        R_Mat_Roll = np.mat([[1, 0, 0],
                             [0, np.cos(roll * (np.pi / 180)), np.sin(roll * (np.pi / 180))],
                             [0, -np.sin(roll * (np.pi / 180)), np.cos(roll * (np.pi / 180))]])

        return R_Mat_Yaw.dot(R_Mat_Pitch.dot(R_Mat_Roll))  # Returns matrix for LLA frame of reference

    def get_world_coords(self, xyzt_cones):  # both inputs are arranged so each row corresponds to the same time.
        xyzt_cones = pd.DataFrame(xyzt_cones)
        imu_t1, imu_t2 = vn200_decoder.get_imu_frame(self, lidar_frame_time=xyzt_cones.iloc[:, 3])
        pd.DataFrame(vn200_decoder.imu_interpolate(self, imu_t1, imu_t2, lidar_frame_time=xyzt_cones.iloc[:, 3]))
        ypr_db = self.interp_imu_frame.iloc[:, 0:3]
        ypr_rtm = [ypr_db.apply(lambda ypr: vn200_decoder.rotation_matrix(self, ypr), axis=1)]

        return xyzt_cones.iloc[:, 0:3].dot(ypr_rtm[0][0])

    def plot_3d(self, figure, xyz_frame, points=0.5):
        axes_str = ['X', 'Y', 'Z']
        axes_limits = [
            [-5, 5],  # X axis range
            [0, 20],  # Y axis range
            [-0.2, 0.6]  # Z axis range
        ]

        point_size = 0.01 * (1. / points)

        def draw_point_cloud(ax, title, axes=[0, 1], xlim3d=None, ylim3d=None, zlim3d=None):
            ax.scatter(*np.transpose(xyz_frame[:, axes]), s=point_size, c=xyz_frame[:, 3], cmap='gray')
            # ax.scatter(*np.transpose(xyz_cones[:, axes]), s=100, c='r' ,marker='^', alpha=.8)
            ax.set_title(title, fontsize=12)
            ax.set_xlabel('{} axis'.format(axes_str[axes[0]]))
            ax.set_ylabel('{} axis'.format(axes_str[axes[1]]))
            if len(axes) > 2:
                ax.set_xlim3d(*axes_limits[axes[0]])
                ax.set_ylim3d(*axes_limits[axes[1]])
                ax.set_zlim3d(*axes_limits[axes[2]])
                ax.set_zlabel('{} axis'.format(axes_str[axes[2]]))
            else:
                ax.set_xlim(*axes_limits[axes[0]])
                ax.set_ylim(*axes_limits[axes[1]])
            # User specified limits
            if xlim3d != None:
                ax.set_xlim3d(xlim3d)
            if ylim3d != None:
                ax.set_ylim3d(ylim3d)
            if zlim3d != None:
                ax.set_zlim3d(zlim3d)

        ax=figure.add_subplot(111)
        plt.gca()
        draw_point_cloud(ax, 'Velodyne scan, XY projection (Z = 0), the car is moving forward (160 deg view)', axes=[0, 1])
        ax.set_aspect('equal')
        plt.draw()
        plt.pause(0.001)
        plt.clf()

    # def animate(i):
    #     ax1 = fig.add_subplot(111, projection='3d')
    #     ax1.clear()
    #
    #     xyz = vn200_decoder.get_world_coords(self, xyzt_cones)
    #
    #     ax1.set_xlim3d([0, 1])
    #     ax1.set_ylim3d([-1, 1])
    #     ax1.set_zlim3d([-1, 1])
    #     ax1.set_title('LIDAR', fontsize=12)
    #     ax1.set_xlabel('{} axis'.format('X'))
    #     ax1.set_ylabel('{} axis'.format('Y'))
    #     ax1.set_zlabel('{} axis'.format('Z'))
    #     ax1.view_init(0, 0)
    #     utilities.set_aspect_equal_3d(ax1)
    #     ax1.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], s=10)
    #
    # def online_feed(self, xyzt_cones):
    #     fig = plt.figure()
    #     ani = animation.FuncAnimation(fig, vn200_decoder.animate, interval=10)  # Live ploting
    #     plt.show()


def main():
    pass


if __name__ == '__main__':
    main()
