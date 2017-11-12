import socket
from control_panel_function import send_data_UDP 
from decryptor import decryptor
import os
import time
def get_LiDAR_data(IP_LiDAR,port_LiDAR,V_matlab,IP_matlab,port_matlab,namber_of_packeges,save_path,file_name,file_nunber):
    """
    function call:
    get_LiDAR_data(IP_LiDAR,port_LiDAR,V_matlab,IP_matlab,port_matlab,...
    namber_of_packeges,save_path,file_name,file_nunber):
        
    function return:
    NONE. The function creates a text file with the recived data.
    
    what dose the function:
    the function recives data from the LiDAR sensor via the rauter,
    decreypt it and creates a text file with the decrypted data. This file is leater 
    being read by the matlab program. 
    """
    time.sleep(1)
    # Create a new socket using the given address family, socket type(AF_INET-default) and protocol number(SOCK_DGRAM- default).
    # creating connection between the computer and the LiDAR:
    sock = socket.socket(socket.AF_INET,            # Internet
                              socket.SOCK_DGRAM)        # UDP
    sock.bind((IP_LiDAR,port_LiDAR))    
        
    # get the data from the lidar:
    data=[]
    for i in range(0,namber_of_packeges):
        # Receive data from the socket. The return value is a pair (string, address) where string is a string representing 
        # the data received and address is the address of the socket sending the data.
        # argument for this func is buffer size
        packet, addr = sock.recvfrom(2000)
        data.append(packet)
            
    # closing the socket:
    sock.close()
    
    # informing that we are decrypting the data:
    V_matlab[12]=2
    send_data_UDP(V_matlab,IP_matlab,port_matlab)    
      
    # decrypting each packege:
    decrypted_data=''
    for i in range(0,namber_of_packeges):
        decrypted_packet = decryptor(data[i])
        decrypted_data   = decrypted_data + decrypted_packet
    
    # printing the data into text file, make sure to change the "save_path":
    file_name = file_name + str(file_nunber) + ".txt"
    save_name = os.path.join(save_path,file_name)
    data_file = open(save_name, "w") 
    data_file.write(decrypted_data)
    data_file.close() 
    
    # informing that we have finished prossesing the data:
    V_matlab[12]=3
    send_data_UDP(V_matlab,IP_matlab,port_matlab)  
