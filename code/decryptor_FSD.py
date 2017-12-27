# The following function is an internal func, which is responsible on the decryption part of the code.

def decryptor_FSD(packet, output_flag, laserID_flag,time_stamp):
    from choose_laserID import choose_laserID

    # converting the packet from it's format to hex numbers
    packet = [format(i, '02x') for i in packet]  # changing each "digit" from the packet to hex format

    # ignoring the end of the packet:
    packet = packet[:len(packet) - 2]

    # extracting the relevant time stamp - last 4 bytes in packet + Reversing+Combining the bytes
    # result in seconds past the hour
    #size = len(packet)
    #time_stamp = packet[size-1] + packet[size-2] + packet[size-3] + packet[size-4]
    #time_stamp = int(time_stamp, 16)  # convert to decimal
    #time_stamp = float(time_stamp)/1000000  # convert from microseconds -> result in seconds past the hour

    # deleting these 4 bytes
    packet = packet[:len(packet) - 4]

    # for this LiDAR phi=[-15,1,-13,-3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15].
    # To convert it to polar spherical coordinate system we did:  phi=90-phii
    # and we got: phi=[105, 89, 103, 93, 101, 85, 99, 83, 97, 81, 95, 79, 93, 77, 91, 75]
    # later we converted it to radians and got:

    phi = [1.8326, 1.5533, 1.7977, 1.6232, 1.7628, 1.4835, 1.7279, 1.4486, 1.693,
           1.4137, 1.6581, 1.3788, 1.6232, 1.3439, 1.5882, 1.309]

    # Theta extraction
    theta = []

    length = len(packet)
    i = 0
    while len(packet) != length - 4 * 12:  # len without: end of packet (6 bytes) + begining of packet (42 bytes) = total 48 bytes
        if packet[i] == 'ee':
            if packet[i - 1] == 'ff':
                az = packet[i + 2] + packet[i + 1]  # Reversing+Combining the bytes
                az = int(az, 16)  # hex to decimal
                az = az * 0.01  # divide by 100
                theta.append(az)
                theta.append(0)  # adding zero to string cause we are missing N+1, N+3, ... azimuth

                del packet[i - 1:i + 3]  # delete 4 bytes at the beginning of each data block
                i = i - 2
        i = i + 1
        # at the end of this while we will have a string of angles every other index (only in the odd indexes)
        # and in the even indexes we will have "0" representing the missing angles

    # Missing theta interpolation
    for i in range(1, 23, 2):
        az = [0, 0, 0]
        az[0] = theta[i - 1]
        az[2] = theta[i + 1] + 360 if az[0] > theta[i + 1] else theta[i + 1]
        az_step = (az[2] - az[0]) * 0.5
        az[1] = az[0] + az_step
        theta[i] = az[1] if az[1] < 360 else az[1] - 360
    theta[-1] = (az[2] + az_step)  # calculating the last angle in the packet


    output_str = choose_laserID(packet, theta, phi, time_stamp, output_flag, laserID_flag)
    return output_str  # returns all the points of the entire packet
