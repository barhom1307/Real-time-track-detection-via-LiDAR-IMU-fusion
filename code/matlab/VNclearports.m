%**************************************************************************
% Function Name  : VNclearports
% Description    : 
%  Closes the connection to all serial ports present on the machine.  This
%  can be used in situations where you opened a serial port, but since lost
%  the handle to the serial port object.
%
% Input(s)       : None    
% Output(s)      : None
%**************************************************************************
%    Examples:
%
%    %Open connection to serial port
%    s = serial('COM8');
%
%    %Just for demonstration purposes we will delete our connection to the
%    %serial port object.  You normally wouldn't do this on purpose.
%    clear s
%
%    %Now we can still close serial port COM8 by calling VNclearports
%    VNclearports();
%
% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function VNclearports()

out = instrfind;

for i=1:length(out)
    fclose(out(i));
end