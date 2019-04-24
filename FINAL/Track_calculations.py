#!/usr/bin/python

import pandas as pd
import itertools

'''
Abstract:   cone_finder func used to extract from full 360 degree lidar frame the exact cone's locations (X,Y,Z).
            work stages:
            1. converting lidar frame into pandas DataFrame (Convenience interest).
            2. filtering requested info from frame and assigning each point it's time stamp.
            3. filtering just the field of view we need (using fov input val).
            4. filtering just the specific lasers we need (using lasers elevation degrees).
            5. locate all points with a difference in the radius column of above 1.5 meters from the previous point.
            6. arranging all these suspicious cone's points in ascending order by their time_stamp value.
Parameters: frame- lidar full frame (360 degrees).
            timestamp_list- frame's time_stamp list.
            fov - field of view value (int).
Returns:    final_cones_pos.values- numpy.ndarray of cone's values (X,Y,Z,time).
            fov_frame.values- numpy.ndarray of lidar fov frame values.
            cones_reflectivity.values - numpy.ndarray of cone's values (X,Y,Z,AZ_deg,ELV_deg,Radius,Reflectivity).
'''
def cone_finder(frame, timestamp_list, fov):
    df = pd.DataFrame(frame, columns=['X', 'Y', 'Z', 'AZ_deg', 'ELV_deg', 'Radius', 'Reflectivity'])
    lasers_deg = [-3, -1, 1, 3]
    lasers_amount = len(lasers_deg)
    df = df.assign(time = timestamp_list)
    fov_frame = df[(df.AZ_deg >= 360-fov/2) | (df.AZ_deg <= fov/2)]
    frame_choosen_lasers = fov_frame[fov_frame['ELV_deg'].isin(lasers_deg)]
    frame_radius_diff = frame_choosen_lasers.filter(items=['Radius']).diff(periods=lasers_amount)
    cone_location = frame_radius_diff.loc[frame_radius_diff['Radius'] < -1.6]
    cones_idx = list(itertools.chain(cone_location.index[:] , cone_location.index[:]+ 16, cone_location.index[:] +16*2))
    cones_idx = list(set([i for i in cones_idx if i <= fov_frame.index[-1]])) #removing duplicates + checking list boundaries
    final_cones_pos = df.iloc[cones_idx,:]
    final_cones_pos = final_cones_pos.query('not(X == 0 & Y == 0 & Z == 0) & (abs(X) <= 2.1 & Y <= 4.9 & Z <= 0.2 & Z >=-0.4)')
    if len(final_cones_pos) == 0:
        return None, None, None
    # reordering the dataframe- the lowest timestamp will be first
    final_cones_pos = final_cones_pos.sort_values(by='time', ascending=True)
    cones_reflectivity = final_cones_pos.filter(items=['X', 'Y', 'Z', 'AZ_deg', 'ELV_deg', 'Radius', 'Reflectivity'])
    final_cones_pos = final_cones_pos.iloc[:,[0,1,2,7]] # X,Y,Z,timestamp of all cones_idx rows
    return final_cones_pos.values, fov_frame.values, cones_reflectivity.values
