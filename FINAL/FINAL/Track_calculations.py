#!/usr/bin/python

import pandas as pd
import itertools
import numpy as np

def cone_finder (frame, timestamp_list, fov):
    df = pd.DataFrame(frame, columns=['X', 'Y', 'Z', 'AZ_deg', 'ELV_deg', 'Radius', 'Reflectivity'])
    lasers_deg = [-3, -1, 1, 3, 5]
    lasers_amount = len(lasers_deg)
    df = df.filter(items=['X', 'Y', 'Z', 'AZ_deg', 'ELV_deg', 'Radius'])
    df = df.assign(time = timestamp_list)
    fov_frame = df[(df.AZ_deg >= 360-fov/2) | (df.AZ_deg <= fov/2)]
    # if len(fov_frame) == 0:   # could be None iff in "get_full_frame" func (LIDAR module) activates "check_last_angle" func
    #     return None, None
    frame_choosen_lasers = fov_frame[fov_frame['ELV_deg'].isin(lasers_deg)]
    frame_radius_diff = frame_choosen_lasers.filter(items=['Radius']).diff(periods=lasers_amount)
    cone_location = frame_radius_diff.loc[frame_radius_diff['Radius'] < -1.5]
    cones_idx = list(itertools.chain(cone_location.index[:] , cone_location.index[:]+ 16, cone_location.index[:] +16*2))
    cones_idx = list(set([i for i in cones_idx if i <= fov_frame.index[-1]])) #removing duplicates + checking list boundaries
    final_cones_pos = df.iloc[cones_idx,[0,1,2,6]] # X,Y,Z,timestamp of all cones_idx rows
    final_cones_pos = final_cones_pos.query('not(X == 0 & Y == 0 & Z == 0) & (abs(X) <= 5 & Y <= 20 & Z <= 0.5)')
    if len(final_cones_pos) == 0:
        return None, None
    # reordering the dataframe- the lowest timestamp will be first
    final_cones_pos = final_cones_pos.sort_values(by='time', ascending=True)
    return final_cones_pos.values, fov_frame.values

