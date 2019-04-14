#!/usr/bin/python

# General system related libraries
import os
import subprocess
import time
import matplotlib.pyplot as plt
import numpy as np
import socket
from struct import unpack
from multiprocessing import Process, Queue, Lock
from signal import signal, SIGTERM
import warnings
import pickle

# global def
NUM_OF_POINTS = 384     # number of points in each lidar packet
STOP_ANGLE = 180        # the required angle to stop scaning each frame
FLAG = True


'''
Abstract:   handler func for recv_worker(class vlp16_online_feeder).
            used as SIGTERM signal handler, in order to assist in dumping all packets raw data into pickle file,
            right after we receive termination signal from the lidar feeder.
Parameters: signum - SIGTERM value
Returns:    None
'''
def handler(signum, frame):
    global FLAG
    FLAG = False


class vlp16_online_feeder:
    def __init__(self, lidar_pkl, addr = ("0.0.0.0", 2368)):
        self.addr = addr
        self.lidar_pkl = lidar_pkl
        self.run()

    '''
    Abstract:   recv_worker func is a thread process which starts it's operation in "run" func.
                it is used as a separate thread in order to establish communication with Lidar and record all raw data
                from the lidar into local packets_list and queue. when it receives termination signal it will dump all recorded
                data to designated pickle file.
    Parameters: queue - container for raw data- live lidar packets
    Returns:    when terminated creates pickle file with all raw data recorded on this session
    '''
    def recv_worker(self, queue, mutex, lidar_pkl):
        self.packets_list = []
        signal(SIGTERM, handler)
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            print("Establish communication with Lidar...")
            sock.bind(self.addr)
            print('Connection Established')
        except OSError: # Handel error if process was left open from last run, close it and retry.
            print("Killing last Lidar process...")
            proc = subprocess.Popen('lsof -i :2368', stdout=subprocess.PIPE, shell=True)
            out = proc.stdout.read()
            out = "".join( chr(x) for x in out)
            out = out.split(sep=' ')
            kill_command = 'kill -9 ' + out[19]
            print(kill_command)
            os.system(kill_command)
            time.sleep(2)
            sock.bind(self.addr)
            print('Connection Established')
        while FLAG:
            # mutex.acquire()
            raw_packet, addr = sock.recvfrom(2000)
            t_stamp = time.time()
            mutex.acquire()
            queue.put((raw_packet,t_stamp))
            self.packets_list.append((raw_packet,t_stamp))
            mutex.release()
        with open(lidar_pkl, 'wb') as lidar_f:
            pickle.dump(self.packets_list, lidar_f)
        print('Terminating LIDAR Connection...')
        os._exit(0)

    '''
    Abstract:   run func creates recv_worker thread, and provide a queue for this thread in order to save live data in
                efficient way.
    Parameters: None
    Returns:    None
    '''
    def run(self):
        self.queue = Queue()
        self.mutex = Lock()
        self.proc = Process(target=self.recv_worker, args=(self.queue, self.mutex, self.lidar_pkl),name='LIDAR_proc')
        self.proc.start()

    '''
    Abstract:   get_frame func used by decoder class to get 76 packets (frame) at once, in order to have full frame of lidar.
    Parameters: None
    Returns:    live UDP lidar raw_packet (binary), and time_stamp of the packet (float).
    '''
    def get_frame(self):
        # print(self.queue.qsize())
        while self.queue.qsize() < 76:
            pass
        self.mutex.acquire()
        raw_frame = []
        while self.queue.qsize():
            raw_packet, t_stamp = self.queue.get()
            raw_frame.append([raw_packet, t_stamp])
        self.mutex.release()
        return raw_frame

    '''
    Abstract:   close_socket func used terminate connection with lidar (close socket).
    Parameters: None
    Returns:    None
    '''
    def close_socket(self):
        self.proc.terminate()


class vlp16_offline_feeder:
    def __init__(self, lidar_f):
        self.packet_list = []
        self.timestamp_list = []
        print("Connecting to offline Lidar feeder...")
        with open(lidar_f, 'rb') as lidar_pkl:
            raw_data = pickle.load(lidar_pkl)
        [(self.packet_list.append(packet), self.timestamp_list.append(t_stamp)) for packet, t_stamp in raw_data]
        frames_amount = np.floor(len(self.packet_list)/76)
        # remove last irrelevant packets from data
        self.packet_list = self.packet_list[:int(frames_amount*76)]
        self.timestamp_list = self.timestamp_list[:int(frames_amount*76)]
        # divide data into 76 packets (size of frame) chunks
        self.packet_list = [self.packet_list[i:i + 76] for i in range(0, len(self.packet_list), 76)]
        self.timestamp_list = [self.timestamp_list[i:i + 76] for i in range(0, len(self.timestamp_list), 76)]

    '''
    Abstract:   get_frame func used by decoder class to get 76 packets at once, in order to have full frame of lidar.
    Parameters: None
    Returns:    pickle raw_packet (binary), and time_stamp of the packet (float).
    '''
    def get_frame(self):
        raw_packet_list, t_stamp_list = self.packet_list.pop(0), self.timestamp_list.pop(0)
        raw_frame = [[raw, t_stamp] for raw, t_stamp in zip(raw_packet_list, t_stamp_list)]
        return raw_frame


# plt.ion()
# fig = plt.figure(1, figsize=[20,10])
# warnings.filterwarnings("ignore",".*GUI is implemented.*")


class vlp16_decoder:
    def __init__(self, packet_feeder):
        self.packet_feeder = packet_feeder
        elevation_deg_vec = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
        self.EL_DEG_REP =  np.repeat(np.array(elevation_deg_vec, ndmin=2), 24, axis=0)
        self.EL_RAD_REP = self.EL_DEG_REP / 180.0 * np.pi
        block_format = 'H' + 'H' + 'HB' * 32  # 2+2+3*32=100
        self.packet_format = '<' + block_format * 12 + 'I' + 'H'  # blocks + timestamp + stuff
        self.prev_az = None
        self.img_idx = 0

    '''
    Abstract:   interpolate_az func used to interpolate the packets azimuth degrees.
                each packets contains only 12 azimuth reading while there are 24 information blocks.
                so 12 measurements are used to interpolate 24 values.
    Parameters: az_per_block - 12 values of azimuth readings.
    Returns:    list of 24 values of azimuth.
    '''
    def interpolate_az(self, az_per_block):
        az_per_block = np.array(az_per_block)
        half_delta_az = np.mean(np.diff(az_per_block) % 360) / 2
        az_per_second_blocks = (az_per_block + half_delta_az) % 360
        return np.hstack(zip(az_per_block, az_per_second_blocks))

    '''
    Abstract:   decode_packet func used to unpack the lidar packet (transform from binary to ascii).
                it will transform the raw data into 24 information blocks, which each will have 16 values of different
                laser elevation. Each laser is contributing a point in the 3d space, meaning we will have 384 points
                in each packet (24 blocks * 16 lasers).
                Each laser has 7 values - X, Y, Z, azimuth degree, elevation degree, Radius, intensity value.
    Parameters: packet - raw lidar data (1206 bytes).
    Returns:    packet_data - decoded packet which contains list of 24 blocks -> each contains list of 16 laser's
                readings -> each contains list of 7 values (as described above).
    '''
    def decode_packet(self, packet):
        res = np.array(unpack(self.packet_format, packet), dtype=np.float32)
        blocks = np.reshape(res[:-2], (12, 2 + 32 * 2))
        az_per_block = []
        RP_list = []
        for block in blocks:
            az_deg = block[1] / 100
            az_per_block.append(az_deg)
            RP = np.reshape(block[2:], (2, 16, 2))
            RP[:, :, 0] *= 0.002
            RP_list.append(RP)
        all_RP = np.concatenate(RP_list)
        R = all_RP[:, :, 0]  # shape = (24, 16)
        P = all_RP[:, :, 1]
        interp_az_deg = self.interpolate_az(az_per_block)
        AZ_deg = np.repeat(np.array(interp_az_deg,ndmin=2).T,16,axis=1)
        interp_az_rad = interp_az_deg / 180.0 * np.pi
        AZ = np.repeat(np.array(interp_az_rad, ndmin=2).T, 16, axis=1)
        EL = self.EL_RAD_REP
        X = R * np.sin(AZ) * np.cos(EL)
        Y = R * np.cos(AZ) * np.cos(EL)
        Z = R * np.sin(EL)
        packet_data = np.stack((X, Y, Z, AZ_deg, self.EL_DEG_REP, R, P), axis=2)
        return packet_data

    # get_next_decoded_packet func currently not in use
    '''
    Abstract:   get_next_decoded_packet func used to withdraw the next packet and time_stamp from the online/offline
                lidar feeder.
    Parameters: None.
    Returns:    time_stamp of packet (float), delta_az - the reminder of the current last azimuth from the last packet's
                azimuth, and packet_data - decoded packet.
    '''
    '''
    def get_next_decoded_packet(self):
        raw_packet, time_stamp = self.packet_feeder.get_packet()
        if raw_packet is None:
            return None, None, None
        packet_data = self.decode_packet(raw_packet)
        if self.prev_az is None:
            self.prev_az = packet_data[0][0][3] #  degree of the first laser set in packet
        # packet_data[-1][0][3] - degree of last laser set in packet
        delta_az = (packet_data[-1][0][3] - self.prev_az) % 360
        self.prev_az = packet_data[-1][0][3]
        return time_stamp, delta_az, packet_data
    '''

    '''
    Abstract:   get_full_frame func used to build full 360 degree lidar frame, which usually contains 76 decoded packets.
                using "while" loop to keep reciveing next decoded packet from fedder, till we have reached 360 cycle.
                we have decided to "cut" each frame at "STOP_ANGLE" (global parameter).
    Parameters: None.
    Returns:    frame- list of all lasers points in frame (76 packets * 384 points) -> each contains 7 values-
                X, Y, Z, azimuth degree, elevation degree, radius, intensity. meaning 2D list [points, 7 values].
                timestamp_list- list of all point's timestamp (as the size of points in frame).
    '''
    def get_full_frame(self):
        raw_data = self.packet_feeder.get_frame()
        raw_data = raw_data[:76]
        decoded_data = [[self.decode_packet(raw), np.repeat(t_stamp, NUM_OF_POINTS)] for raw, t_stamp in raw_data]
        decoded_frame = np.concatenate([i[0] for i in decoded_data])
        packets_timestamp_list = [i[1] for i in decoded_data]
        frame = np.reshape(decoded_frame, (decoded_frame.shape[0]*decoded_frame.shape[1],decoded_frame.shape[2]))
        timestamp_list = np.reshape(packets_timestamp_list, (len(packets_timestamp_list)*NUM_OF_POINTS,1))
        # removing last 224 points of frame in order to have exact 360 degree cycle
        frame = frame[:-224]
        timestamp_list = timestamp_list[:-224]
        return frame, timestamp_list

    '''
    Abstract:   decode_N_packets func used to build lidar frame from specific requested number of packets.
                please note - it isn't used in our program at all!!!
    Parameters: N- number of packets requested to build the frame from.
    Returns:    packets_data_list- list of all lasers points in requested number of packets.
                timestamp_list- list of all point's timestamp.
    '''
    def decode_N_packets(self, N):
        total_angle = 0
        packets_data_list = []
        packets_timestamp_list = []
        for ind in range(int(N)):
            timestamp, delta_az, packet_data = self.get_next_decoded_packet()
            if packet_data is None:
                return None, None, None
            total_angle += delta_az
            packets_data_list.append(packet_data)
            packets_timestamp_list.append(timestamp)
        return np.concatenate(packets_data_list), packets_timestamp_list

    '''
    Abstract:   plot_2d func used to plot XY lidar frame (Z=0) frame.
                the plot will include full 180 degree front view, and on top on it a scatter of the cones detected in red.
    Parameters: velo_frame- lidar full frame.
                xyz_cones- cones X & Y values.
                timeStamp- frame's time_stamp in order to attach it to plot's title.
    Returns:    None.
    '''
    def plot_2d(self, velo_frame, xyz_cones, timeStamp):
        axes_str = ['X', 'Y', 'Z']
        axes_limits = [
            [-3, 3],  # X axis range
            [0, 12],  # Y axis range
            [-0.2, 0.6]  # Z axis range
        ]

        def draw_point_cloud(ax, title, axes=[0, 1], xlim3d=None, ylim3d=None, zlim3d=None):
            try:
                ax.scatter(*np.transpose(velo_frame[:, axes]), s=5, c=velo_frame[:, 3], cmap='Set1')
                ax.scatter(*np.transpose(xyz_cones[:, axes]), s=100, c='r', marker='^', alpha=.8)
            except TypeError:
                pass
            fig.text(0.5, 0.95, title, fontsize=14, horizontalalignment='center', verticalalignment='top', transform=ax.transAxes)

            ax.set_xlabel('{} axis'.format(axes_str[axes[0]]))
            ax.set_ylabel('{} axis'.format(axes_str[axes[1]]))
            ax.set_xlim(*axes_limits[axes[0]])
            ax.set_ylim(*axes_limits[axes[1]])
            # User specified limits
            if xlim3d != None:
                ax.set_xlim3d(xlim3d)
            if ylim3d != None:
                ax.set_ylim3d(ylim3d)
        ax  = fig.add_subplot(111)
        #plt.gca()
        title_txt = 'Frame timeStamp- ' + str(timeStamp) + ' sec'
        #draw_point_cloud(ax, 'Velodyne scan, XY projection (Z = 0), the car is moving forward (160 deg view)', axes=[0, 1])
        draw_point_cloud(ax, title_txt, axes=[0, 1])
        ax.set_aspect('equal')
        plt.draw()
        # images_path = '/home/avinoam/Desktop/img_13_04_19'
        # plt.savefig(images_path + "/image_{}.png".format(self.img_idx))
        # self.img_idx += 1
        plt.pause(0.001)
        plt.clf()


def main():
    print('Running LIDAR script')


if __name__ == '__main__':
    main()