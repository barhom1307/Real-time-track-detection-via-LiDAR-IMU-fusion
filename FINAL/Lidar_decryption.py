import numpy as np
from struct import unpack
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import socket
from multiprocessing import Process, Queue
import time
import pcap
from utilities import print_progress, set_aspect_equal_3d
from moviepy.editor import ImageSequenceClip
import pandas as pd
from cone_finder_new import cone_finder
import warnings


class vlp16_live_listner:
    def __init__(self, addr = ("0.0.0.0", 2368)):
        self.addr = addr
        self.run()

    def recv_worker(self, queue_out):
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.bind(self.addr)
        prev_az = 0  # Not correct but not very important
        while True:
            raw_packet, addr = sock.recvfrom(2000)
            queue_out.put((raw_packet,"{0:.5f}".format(time.time())))

    def run(self):
        self.queue = Queue()
        self.proc = Process(target=self.recv_worker, args=(self.queue,))
        self.proc.start()

    def get_packet(self):
        packet, time_stamp = self.queue.get()
        return packet, time_stamp

    def close_socket(self):
        self.proc.terminate()


class vlp16_offline_simulator:
    def __init__(self, pcap_fd):
        res = [i for i in pcap_fd]
        res = np.reshape(res,(len(res),2))
        res = res[:,1]
        self.packets = [''.join(x) for x in res]
        self.time_stamp = 0

    def get_packet(self):
        if len(self.packets) > 0:
            return self.packets.pop(0), self.time_stamp     #  time_stamp is 0 because we will take the time val from txt file
        else:
            return None


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
        #timestamp = res[-2] * 1e-6  # convert timestamp from microseconds to seconds
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
        #return timestamp, packet_data  #YAPPPPPPPPPP
        return packet_data

    def get_next_decoded_packet(self, live_flag, pcap_fd):
        raw_packet, time_stamp = self.packet_feeder.get_packet()
        if live_flag == True and raw_packet is not None:
            pcap_fd.write(((0,0,1248),raw_packet))
        if raw_packet is None:
            return None, None, None
        packet_data = self.decode_packet(raw_packet)
        if self.prev_az is None:
            self.prev_az = packet_data[0][0][3] #  degree of the first laser set in packet
        delta_az = (packet_data[-1][0][3] - self.prev_az) % 360     #  packet_data[-1][0][3] - degree of last laser set in packet
        self.prev_az = packet_data[-1][0][3]
        return time_stamp, delta_az, packet_data

    def get_full_frame(self, live_flag, pcap_fd):
        total_angle = 0
        packets_data_list = []
        packets_timestamp_list = []
        while total_angle < 360:
            timestamp, delta_az, packet_data = self.get_next_decoded_packet(live_flag, pcap_fd)
            if packet_data is None:
                return None, None, None
            total_angle += delta_az
            packets_data_list.append(packet_data)
            packets_timestamp_list.append(timestamp)
        return np.concatenate(packets_data_list), packets_timestamp_list, total_angle

    def decode_N_packets(self, N, live_flag, pcap_fd):
        total_angle = 0
        packets_data_list = []
        packets_timestamp_list = []
        for ind in range(int(N)):
            timestamp, delta_az, packet_data = self.get_next_decoded_packet(live_flag, pcap_fd)
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



if __name__ == '__main__':

    print('Please choose your desired working mode from the following:')
    print('Mode 0 - live streaming, REC pcap file')
    print('Mode 1 - offline decrypt N frames from pcap -> output to csv file')
    print('Mode 2 - copy part of pcap file to new pcap file')
    print('Mode 3 - offline decryption from pcap file + cone image')
    print('Mode 4 - real time cone detection -> output will be (x,y,z) of cones')
    print('Mode 5 - offline decrypt N frames + cone detection -> output to csv file [(x,y,z) of cones in each frame]')
    mode = int(input('Please type your choice :  '))
    file_path = "/home/avinoam/Desktop/"
    packets_timestamp_list = []
    total_angle = []
    frame = []
    feeder = 0

    if mode == 0: # live streaming - recording data to pcap file

        timeout = int(input('Please enter a desired recording time (in seconds): ')) # seconds

        print("Establish communication with Lidar...")
        feeder = vlp16_live_listner()
        decoder = vlp16_decoder(feeder)

        live_flag = True
        pcap_name = file_path + "raw_Lidar_" + time.strftime("%X_%d.%m.%y") + '.pcap'
        pcap_fd = pcap.open(pcap_name, 'w')
        file_time = file_path + "time_stamp_" + time.strftime("%X_%d.%m.%y") + '.txt'


        N_frames =  10*timeout    #  10 frames per sec (velo working freq is 10Hz)
        for frame in range(N_frames):
            print(frame)
            frame, packets_timestamp_list, total_angle = decoder.get_full_frame(live_flag,pcap_fd)
            print(frame)
            frame = np.reshape(frame, (frame.shape[0]*frame.shape[1],frame.shape[2]))
            #frames_list.append(frame)
            packets_timestamp_list = np.reshape(packets_timestamp_list, (len(packets_timestamp_list),1))
            with open(file_time, 'a') as a:
                for i in range(len(packets_timestamp_list)):
                    a.write('\n'.join(map(str,packets_timestamp_list[i])) + '\n')

        time.sleep(1) # giving time to write the last packet
        feeder.close_socket()

        print ('Done!!! - {} frames has been recorded'.format(N_frames))


    if mode == 1: # offline - reading from pcap file and decrypt frames into csv file

        read_file = pcap.open("/home/avinoam/Desktop/raw_Lidar_21:25:41_19.08.18.pcap", 'r')  # need to enter desired file
        time_file = open("/home/avinoam/Desktop/time_stamp_21:25:41_19.08.18.txt" , 'r')  # need to enter desired file
        feeder = vlp16_offline_simulator(read_file)
        print("Reading packets from pcap file...")

        output_file = "/home/avinoam/Desktop/decrypted_data" + time.strftime("%X_%d.%m.%y") + '.csv'
        live_flag = False
        pcap_fd = 0 # pcap not in use
        decoder = vlp16_decoder(feeder)

        frames_amount = int(input('Please enter the desired number of frames to decrypt:  '))
        for i in range(frames_amount):
            print_progress(i, frames_amount-1)

            time_list = []
            frame, packets_timestamp_list, total_angle = decoder.get_full_frame(live_flag,pcap_fd)
            frame = np.reshape(frame, (frame.shape[0]*frame.shape[1],frame.shape[2]))
            for i in range(len(frame)/384):
                time_val = np.array(float(time_file.readline()))
                time_val = np.repeat(time_val,384)
                time_list.append(time_val)
            time_list= np.reshape(time_list,(len(time_list)*384,1))
            new_frame = np.concatenate((frame,time_list),axis=1)

            data_to_csv = pd.DataFrame(new_frame)
            data_to_csv.to_csv(output_file, header= ['X', 'Y', 'Z', 'AZ_deg', 'ELV_deg', 'Radius', 'Reflectivity', 'Time_stamp'], mode= 'a')


        print ("Done!!!")


    if mode == 2:   # copy part of pcap file to new pcap
        read_file = pcap.open("/home/avinoam/Desktop/raw_Lidar_16:30:44_26.08.18.pcap", 'r')  # need to enter desired file
        write_file = pcap.open("/home/avinoam/Desktop/raw_lidar_new.pcap", 'w')
        j = 1
        for i in read_file:
            if j <= 83600:  # enter number of packet (within pcap file)
                write_file.write(((0,0,1248),i[1]))
            j= j+1



    if mode == 3:   # offline decryption test + cone image
        read_file = pcap.open("/home/avinoam/Desktop/raw_Lidar_16:02:08_15.06.18.pcap", 'r')  # need to enter desired file
        feeder = vlp16_offline_simulator(read_file)
        print("Reading packets from pcap file...")

        live_flag = False
        pcap_fd = 0 # pcap not in use
        decoder = vlp16_decoder(feeder)

        N_frames = int(input('Please enter the desired number of frames to decrypt:  '))

        plt.ion()
        fig = plt.figure()
        warnings.filterwarnings("ignore",".*GUI is implemented.*")

        for frm in range(N_frames):
            print_progress(frm, N_frames-1)
            frame, packets_timestamp_list, total_angle = decoder.get_full_frame(live_flag,pcap_fd)
            frame = np.reshape(frame, (frame.shape[0]*frame.shape[1],frame.shape[2]))
            xyz_cones , frame_160deg = cone_finder(frame)

            decoder.plot_3d(fig, frame_160deg, xyz_cones)


    if mode == 4:   # real time cone detection -> output will be (x,y,z) of cones

        timeout = int(input('Please enter a desired recording time (in seconds): ')) # seconds

        print("Establish communication with Lidar...")
        feeder = vlp16_live_listner()
        decoder = vlp16_decoder(feeder)

        live_flag = True
        pcap_name = file_path + "raw_Lidar_" + time.strftime("%X_%d.%m.%y") + '.pcap'
        pcap_fd = pcap.open(pcap_name, 'w')
        file_time = file_path + "time_stamp_" + time.strftime("%X_%d.%m.%y") + '.txt'
        cones_file = "/home/avinoam/Desktop/cones_" + time.strftime("%X_%d.%m.%y") + '.csv'


        N_frames =  timeout * 10    #  10 frames per sec (velo working freq is 10Hz)
        #N_frames = 1
        for frame in range(N_frames):
            frame, packets_timestamp_list, total_angle = decoder.get_full_frame(live_flag,pcap_fd)
            frame = np.reshape(frame, (frame.shape[0]*frame.shape[1],frame.shape[2]))
            xyz_cones , frame_160deg = cone_finder(frame)
            packets_timestamp_list = np.reshape(packets_timestamp_list, (len(packets_timestamp_list),1))
            with open(file_time, 'a') as a:
                for i in range(len(packets_timestamp_list)):
                    a.write('\n'.join(map(str,packets_timestamp_list[i])) + '\n')
            cone_to_csv = pd.DataFrame(xyz_cones)
            cone_to_csv.to_csv(cones_file, header= ['X', 'Y', 'Z'], mode= 'a')


        time.sleep(1) # giving time to write the last packet
        feeder.close_socket()

        print ('Done!!! - {} frames has been recorded'.format(N_frames))

    if mode == 5: #offline decryption - output is csv file of (x,y,z) of cones in each frame

        read_file = pcap.open("/home/avinoam/Desktop/raw_lidar_new.pcap", 'r')  # need to enter desired file
        cones_file = "/home/avinoam/Desktop/cones_" + time.strftime("%X_%d.%m.%y") + '.csv'
        feeder = vlp16_offline_simulator(read_file)
        print("Reading packets from pcap file...")

        live_flag = False
        pcap_fd = 0 # pcap not in use
        decoder = vlp16_decoder(feeder)
        N_frames = int(input('Please enter the desired number of frames to decrypt:  '))

        for frm in range(N_frames):
            idx_list = []
            print_progress(frm, N_frames-1)
            frame, packets_timestamp_list, total_angle = decoder.get_full_frame(live_flag,pcap_fd)
            frame = np.reshape(frame, (frame.shape[0]*frame.shape[1],frame.shape[2]))
            xyz_cones , frame_160deg = cone_finder(frame)
            #print(xyz_cones)
            cone_to_csv = pd.DataFrame(xyz_cones)
            cone_to_csv.to_csv(cones_file, header= ['X', 'Y', 'Z'], mode= 'a')
