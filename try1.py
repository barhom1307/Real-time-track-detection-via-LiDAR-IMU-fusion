from get_LiDAR_data_via_rauter import get_LiDAR_data

IP_LiDAR        = "0.0.0.0"
port_LiDAR      = 2368

V_matlab=[0,1,2,3,4,5,6,7,8,9,10,11,12,13]

IP_matlab='127.0.0.1'
port_matlab=8850
namber_of_packeges=75

save_path='C:/Users/Tom/Desktop'
file_nunber='0'
file_name='data_file_'


get_LiDAR_data(IP_LiDAR,port_LiDAR,V_matlab,IP_matlab,port_matlab,namber_of_packeges,save_path,file_name,file_nunber)
    
    



    