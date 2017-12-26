# The following function will modify the output string as the user requested.
# it will do so by compering the output_flag (inserted by the user in !!!!!decimal number!!!!) to bitwise check.
# The user needs to insert his request by the following order (from left to right):
# <x-axis> <y-axis> <z-axis> <radius> <azimuth> <reflectivity> <time_stamp>
# meaning that we will have a total of 7 bits, which each of these bits will inform the following function
# if the user requested to see the specific data(described by that bit) on the output string.

def choose_output(x, y, z, r, az, reflectivity, time, output_flag):

    output_str = ''
    # means that <x-axis> bit is "on"
    if output_flag & 64 == 64:
        output_str = output_str + "%12s" % x
    # means that <y-axis> bit is "on"
    if output_flag & 32 == 32:
        output_str = output_str + "%12s" % y
    # means that <z-axis> bit is "on"
    if output_flag & 16 == 16:
        output_str = output_str + "%12s" % z
    # means that <radius> bit is "on"
    if output_flag & 8 == 8:
        output_str = output_str + "%12s" % r
    # means that <azimuth> bit is "on"
    if output_flag & 4 == 4:
        output_str = output_str + "%12s" % az
    # means that <reflectivity> bit is "on"
    if output_flag & 2 == 2:
        output_str = output_str + "%12s" % reflectivity
    # means that <time_stamp> bit is "on"
    if output_flag & 1 == 1:
        output_str = output_str + "%20s" % time
    output_str = output_str + '\n'
    return output_str
