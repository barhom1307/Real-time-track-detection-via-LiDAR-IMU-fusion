# The following function will check which lasers the user decided to enable on his output file.
# The user needs to enter the correct laserID_flag in accordance to his choice.
# The options are as follows:   1.ALL_LASERS- will print data from all 16 IR lasers.
#                               2.LASER_1_and_14- will print data from 2 IR lasers- ID_1 - 1 degree, ID_14 - (-1) degree
#                               3.LASER_1- will print data only from one IR laser- ID_1 - 1 degree.
#                               4. any other input will print error message to user.

def choose_laserID(packet, theta, phi, time_stamp, output_flag, laserID_flag):
    from math import cos, sin, pi
    from choose_output import choose_output
    output = ''
    # laserID_flag define
    ALL_LASERS = 0
    LASER_1_and_14 = 1
    LASER_1 = 2

    #laserID_flag = ALL_LASERS -> print data from all 16 IR lasers
    if laserID_flag == ALL_LASERS:
        # Radius extraction and point calculation
        N = 16 * 3  # the number of bytes in half data block: 16 channels, 3 bytes for each channel
        M = 3  # the number of bytes per one channel
        for j in range(24):  # go over 24 mini data block (each data block is devided to 2 mini data block)
            for i in range(16):  # go over the channels

                # calculating the radius:
                r = packet[j * N + i * M + 1] + packet[j * N + i * M]  # Reversing + Combining the bytes
                r = int(r, 16)  # hex to decimal
                r = r * 0.002  # Multiply by 2mm

                reflectivity = packet[j * N + i * M + 2]  # extracting the reflectivity data
                reflectivity = int(reflectivity, 16)  # hex to decimal
                reflectivity = str(reflectivity)

                time = (0.00005526 * j) + (0.0000023 * i)  # calculating the exact point time in seconds
                time = time + time_stamp  # result in seconds past the hour
                time = str(round(time,4))

                az = theta[j]
                az = az * pi / 180  # calculating the azimuth

                x = str(round(r * cos(az) * sin(phi[i]), 2))
                y = str(round(-r * sin(az) * sin(phi[i]), 2))
                z = str(round(r * cos(phi[i]), 2))
                az = str(round((az*180)/pi, 2))
                r = str(round(r, 2))

                # creating string that will hold all the data and will be printed to text file later in the program:
                output = output + choose_output(x, y, z, r, az, reflectivity, time, output_flag)

        return output

    #laserID_flag = LASER_1_and_14 -> print data from 2 IR lasers- ID_1 - 1 degree, ID_14 - (-1) degree
    elif laserID_flag == LASER_1_and_14:
        # Radius extraction and point calculation
        N = 16 * 3  # the number of bytes in half data block: 16 channels, 3 bytes for each channel
        M = 3  # the number of bytes per one channel
        for j in range(24):  # go over 24 mini data block (each data block is devided to 2 mini data block)
            for i in range(16):  # go over the channels
                if i != 1 and i != 14: continue
                # calculating the radius:
                r = packet[j * N + i * M + 1] + packet[j * N + i * M]  # Reversing + Combining the bytes
                r = int(r, 16)  # hex to decimal
                r = r * 0.002  # Multiply by 2mm

                reflectivity = packet[j * N + i * M + 2]  # extracting the reflectivity data
                reflectivity = int(reflectivity, 16)  # hex to decimal
                reflectivity = str(reflectivity)

                time = (0.00005526 * j) + (0.0000023 * i)  # calculating the exact point time in seconds
                time = time + time_stamp  # result in seconds past the hour
                time = str(round(time,3))

                az = theta[j]
                az = az * pi / 180  # calculating the azimuth

                x = str(round(r * cos(az) * sin(phi[i]), 2))
                y = str(round(-r * sin(az) * sin(phi[i]), 2))
                z = str(round(r * cos(phi[i]), 2))
                az = str(round((az * 180) / pi, 2))
                r = str(round(r, 2))

                # creating string that will hold all the data and will be printed to text file later in the program:
                output = output + choose_output(x, y, z, r, az, reflectivity, time, output_flag)

        return output

    # laserID_flag = LASER_1 -> print data from only one IR laser- ID_1 - 1 degree
    elif laserID_flag == LASER_1:
        # Radius extraction and point calculation
        N = 16 * 3  # the number of bytes in half data block: 16 channels, 3 bytes for each channel
        M = 3  # the number of bytes per one channel
        for j in range(24):  # go over 24 mini data block (each data block is devided to 2 mini data block)
            for i in range(16):  # go over the channels
                if i != 1 : continue
                # calculating the radius:
                r = packet[j * N + i * M + 1] + packet[j * N + i * M]  # Reversing + Combining the bytes
                r = int(r, 16)  # hex to decimal
                r = r * 0.002  # Multiply by 2mm

                reflectivity = packet[j * N + i * M + 2]  # extracting the reflectivity data
                reflectivity = int(reflectivity, 16)  # hex to decimal
                reflectivity = str(reflectivity)

                time = (0.00005526 * j) + (0.0000023 * i)  # calculating the exact point time in seconds
                time = time + time_stamp  # result in seconds past the hour
                time = str(round(time, 3))

                az = theta[j]
                az = az * pi / 180  # calculating the azimuth

                x = str(round(r * cos(az) * sin(phi[i]), 2))
                y = str(round(-r * sin(az) * sin(phi[i]), 2))
                z = str(round(r * cos(phi[i]), 2))
                az = str(round((az * 180) / pi, 2))
                r = str(round(r, 2))

                # creating string that will hold all the data and will be printed to text file later in the program:
                output = output + choose_output(x, y, z, r, az, reflectivity, time, output_flag)

        return output

    # if we arrived here, meaning that there was an invalid input on laserID_flag.
    else:
        output = "Please enter valid laserID_flag input.\n"
        return output
