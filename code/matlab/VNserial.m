%**************************************************************************
% Function Name  : VN_Serial 
% Description    : 
%  Opens a new serial port for communication with a VectorNav sensor.  If
%  a baud rate is not given, the sensor is assumed to be set to 115200,
%  which is the factory default baud rate.  If the device is set to use any
%  other baud rate, then the baud rate must be given as an input parameter
%  to this function. Once connected all asynchronous outputs will be turned
%  off.
%
% Input(s)       : comPort  -> COM port name or number ('COM8' or 8)
%                : BaudRate -> optional baud rate
% Output(s)      : s        -> serial port object
%**************************************************************************
% Examples:
%
%   s = VNserial('COM8');
%   VNwriteRegister(s, 23, [1 0 0 0 1 0 0 0 1 0 0 0]);
%   fclose(s);
%
% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function s = VNserial(comPort, BaudRate)

%Convert input to string
if isstr(comPort) == 0
    comPort = ['COM' num2str(comPort)];
end

%Check to see if COM port is already open, if so then close COM port.
ports = instrfind;
for i=1:length(ports)
    if strcmp(ports(i).Port, comPort) == 1
        fclose(ports(i));
    end
end

if ~exist('BaudRate','var')
  BaudRate = 115200;
end

%Create the serial port
s = serial(comPort, 'BaudRate', BaudRate);

%Create the serial port
fopen(s);
VNprintf(s, 'VNWRG,6,0');
VNprintf(s, 'VNWRG,7,200');
pause(0.25);
VNclearbuffer(s);
