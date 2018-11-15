#!/usr/bin/python

import numpy as np
import socket
import subprocess
import os, time
from multiprocessing import Process, Queue
import pytictoc
from Track_calculations import cone_finder, extrapolate_points
from struct import unpack
from scapy.all import IP, UDP, wrpcap, Ether, rdpcap


NUM_OF_POINTS = 384     # number of points in each lidar packet
STOP_ANGLE = 180        # the required angle to stop scaning each frame


class vlp16_online_feeder:
    def __init__(self, lidar_pcap, FOV, addr = ("0.0.0.0", 2368)):
        self.addr = addr
        self.lidar_f = lidar_pcap
        self.FOV = FOV
        self.packets = []
        self.time_stamp = []
        self.ether = Ether()
        self.ip = IP(src="192.168.1.201", dst="255.255.255.255")
        self.udp = UDP(sport=2368, dport=2368)
        self.run()

    def recv_worker(self, queue ,lidar_f, FOV):
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
            kill_command = 'kill ' + out[19]
            print(kill_command)
            os.system(kill_command)
            time.sleep(2)
            sock.bind(self.addr)
            print('Connection Established')

        #prev_az = 0  # Not correct but not very important
        # with open(lidar_f, 'ab') as lidar_fd:
        while True:
        # for i in range(10):
            raw_packet, addr = sock.recvfrom(2000)
            # 1102 & 1103 are the bytes index of the last degree in packet
            # last_deg = int.from_bytes(raw_packet[1102:1104],byteorder='little')
            # if (last_deg >= 36000-(FOV/2)*100) or (last_deg <= (FOV/2)*100):
            t_stamp = time.time()
            queue.put((raw_packet,t_stamp))
            # packets.append(packet)
            # wrpcap(lidar_f, packet, append=True)

    def run(self):
        self.queue = Queue()
        self.proc = Process(target=self.recv_worker, args=(self.queue, self.lidar_f, self.FOV ),name='LIDAR_proc')
        self.proc.start()

    def get_packet(self):
        packet, time_stamp = self.queue.get()
        self.packets = np.append(self.packets, packet)
        self.time_stamp = np.append(self.time_stamp, time_stamp)
        return packet, time_stamp

    def send_to_pcap(self):
        packets = []
        pkt_list = self.packets.tolist()
        time_list = self.time_stamp.tolist()
        while len(self.packets) is not 0:
            pkt = self.ether/self.ip/self.udp/pkt_list.pop(0)
            pkt.time = time_list.pop(0)
            packets = np.append(packets, pkt)
        wrpcap(self.lidar_f, packets)

    def close_socket(self):
        print('Terminating LIDAR Connection...')
        self.proc.terminate()
        self.send_to_pcap()
        time.sleep(2)


class vlp16_offline_feeder:
    def __init__(self, lidar_pcap):
        self.LIDAR_PCAP = lidar_pcap
        packet_array = []
        timestamp_array = []
        print("Connecting to offline Lidar feeder...")
        with open(self.LIDAR_PCAP, 'r') as lidar_f:
            while True:
                try:
                    packet, timestamp = pcap.rdpcap(lidar_f)
                    packet_array = np.r_[packet_array, packet]
                    timestamp_array = np.r_[timestamp_array,timestamp]
                except EOFError:
                    break
        self.packet_list = packet_array.tolist()
        self.timestamp_list = timestamp_array.tolist()

    def get_packet(self):
        if len(self.packet_list) > 0:
            return self.packet_list.pop(0), self.timestamp_list.pop(0)
        else:
            return None, None


class vlp16_decoder:
    def __init__(self, packet_feeder):
        self.packet_feeder = packet_feeder
        elevation_deg_vec = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
        self.EL_DEG_REP =  np.repeat(np.array(elevation_deg_vec, ndmin=2), 24, axis=0)
        self.EL_RAD_REP = self.EL_DEG_REP / 180.0 * np.pi
        block_format = 'H' + 'H' + 'HB' * 32  # 2+2+3*32=100
        self.packet_format = '<' + block_format * 12 + 'I' + 'H'  # blocks + timestamp + stuff
        self.prev_az = None

    def interpulate_az(self, az_per_block):
        az_per_block = np.array(az_per_block)
        half_delta_az = np.mean(np.diff(az_per_block) % 360) / 2
        az_per_second_blocks = (az_per_block + half_delta_az) % 360
        return np.hstack(zip(az_per_block, az_per_second_blocks))

    def decode_packet(self, packet):
        res = np.array(unpack(self.packet_format, packet), dtype=np.float32)
        blocks = np.reshape(res[:-2], (12, 2 + 32 * 2))
        az_per_block = []
        RP_list = []
        for block in blocks:
            az_deg = block[1] * 0.01
            az_per_block.append(az_deg)
            RP = np.reshape(block[2:], (2, 16, 2))
            RP[:, :, 0] *= 0.002
            RP_list.append(RP)

        all_RP = np.concatenate(RP_list)
        R = all_RP[:, :, 0]  # shape = (24, 16)
        P = all_RP[:, :, 1]
        interp_az_deg = self.interpulate_az(az_per_block)
        AZ_deg = np.repeat(np.array(interp_az_deg,ndmin=2).T,16,axis=1)
        interp_az_rad = interp_az_deg / 180.0 * np.pi
        AZ = np.repeat(np.array(interp_az_rad, ndmin=2).T, 16, axis=1)
        EL = self.EL_RAD_REP
        X = R * np.sin(AZ) * np.cos(EL)
        Y = R * np.cos(AZ) * np.cos(EL)
        Z = R * np.sin(EL)
        packet_data = np.stack((X, Y, Z, AZ_deg, self.EL_DEG_REP, R, P), axis=2)
        return packet_data

    # def get_decoded_packet(self):
    #     raw_packet, time_stamp = self.packet_feeder.get_packet()
    #     if raw_packet is None:
    #         return None, None
    #     packet_data = self.decode_packet(raw_packet)
    #     return time_stamp, packet_data

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

    def check_last_angle(self):
        if self.prev_az == None:    #recording the first packet - avoid checking degree
            return False
        #checking if the last_angle of the packet is in the range of +-2.5 deg from STOP_ANGLE val
        if self.prev_az >= STOP_ANGLE-2.5 and self.prev_az <= STOP_ANGLE+2.5:
            return True
        else:
            return False

    def get_full_frame(self):
        total_angle = 0
        packets_data_list = []
        packets_timestamp_list = []
        while total_angle < 360 : # degrees
            timestamp, delta_az, packet_data = self.get_next_decoded_packet()
            if packet_data is None:
                return None, None
            total_angle += delta_az
            packets_data_list.append(packet_data)
            timestamp = np.repeat(timestamp, NUM_OF_POINTS)
            packets_timestamp_list.append(timestamp)
            # making sure that we are stopping the "frame-decoding" each time at the same degree (STOP_ANGLE)
            # if self.check_last_angle() == True:
            #     break
        return np.concatenate(packets_data_list), packets_timestamp_list

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
        return np.concatenate(packets_data_list), packets_timestamp_list, total_angle

    def plot_3d(self,figure, velo_frame, xyz_cones, points=0.5):

        axes_str = ['X', 'Y', 'Z']
        axes_limits = [
            [-5, 5],  # X axis range
            [0, 20],  # Y axis range
            [-0.2, 0.6]  # Z axis range
        ]

        point_size = 0.01 * (1. / points)

        def draw_point_cloud(ax, title, axes=[0, 1], xlim3d=None, ylim3d=None, zlim3d=None):
            ax.scatter(*np.transpose(velo_frame[:, axes]), s=point_size, c=velo_frame[:, 3], cmap='gray')
            ax.scatter(*np.transpose(xyz_cones[:, axes]), s=100, c='r' ,marker='^', alpha=.8)
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

        ax  = figure.add_subplot(111)
        plt.gca()
        draw_point_cloud(ax, 'Velodyne scan, XY projection (Z = 0), the car is moving forward (160 deg view)', axes=[0, 1])
        ax.set_aspect('equal')
        plt.draw()
        plt.pause(0.001)
        plt.clf()

def main():

    FOV = 160
    LIDAR_PCAP = '/home/avinoam/Desktop/lidar_raw.pcap'
    feeder = vlp16_online_feeder(LIDAR_PCAP, FOV)
    lidar_decoder = vlp16_decoder(feeder)
    # time.sleep(5)
    # a = rdpcap(LIDAR_PCAP)
    # time_stamp = []
    # packets = []
    # for pkt in a:
    #     time_stamp = np.append(time_stamp, pkt.time)
    #     packets = np.append(packets, pkt.load)
    # time_stamp = time_stamp.tolist()
    # packets = packets.tolist()
    # print(len(time_stamp), type(time_stamp), time_stamp.pop(0), time_stamp.pop(0))
    # print(len(packets), type(packets), packets.pop(0))


    # #N_frames =  timeout * 10    #  10 frames per sec (velo working freq is 10Hz)
    N_frames = 100
    t = pytictoc.TicToc()
    t.tic()
    for packet in range(N_frames):
    #     # utilities.print_progress(packet, N_packets-1)
        decoded_frame, time_stamp = lidar_decoder.get_full_frame()
        frame = np.reshape(decoded_frame, (decoded_frame.shape[0]*decoded_frame.shape[1],decoded_frame.shape[2]))
        timestamp_list = np.reshape(time_stamp, (len(time_stamp)*NUM_OF_POINTS,1))
    #     # df_packet = pd.DataFrame(new_packet)
    #     xyz_cones = cone_finder(frame, timestamp_list, FOV)
    #     # if np.all(xyz_cones) == None:
    #     #     continue
    #     # else:
    #     #     # print(xyz_cones)
    #     #     xyz_world = feeder.get_world_coordinate(xyz_cones, time_stamp)
    #     #     cone_to_csv = pd.DataFrame(xyz_cones)
    #     #     cone_to_csv.to_csv(CONES_CSV, header= ['X', 'Y', 'Z'], mode= 'a')
    t.toc()
    feeder.close_socket()
    # pr.disable()
    # pr.print_stats()

if __name__ == '__main__':
    main()