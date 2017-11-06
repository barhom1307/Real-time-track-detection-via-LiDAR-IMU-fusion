import socket 
from control_panel_function import send_data_UDP 
import time

def get_instructions(IP_instructions_matlab,port_instructions_matlab,IP_matlab,port_matlab,V_matlab):   
    
    # send to display in the matlab:
    send_data_UDP(V_matlab,IP_matlab,port_matlab) 


    Buffer_size=2048
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind((IP_matlab,port_instructions_matlab))
    except:
        time.sleep(1)
        s.bind((IP_matlab,port_instructions_matlab))
        
    s.listen(1)
    s.settimeout(10)
    try: 
        connection, addr = s.accept() 
        connection.setblocking(1)
                 
    except socket.timeout:
        send_data_UDP(V_matlab,IP_matlab,port_matlab)
        connection, addr = s.accept() 
        connection.setblocking(1)
        
    recived_instructions = connection.recv(Buffer_size)  
    
    # convert data from string to numbers
    recived_instructions=recived_instructions.split()
    
    recived_instructions = map(float,recived_instructions)
    
    s.close()

    return recived_instructions