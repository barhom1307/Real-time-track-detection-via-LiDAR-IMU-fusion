%**************************************************************************
% Function Name  : VNprintf
% Description    : 
%  VNprintf prints the given text out to the VN-100 on the serial object
%  define by serial. Both the header ($) and checksum (*XX) will be added
%  to the given string and sent to the device.
%
% Input(s)       : VNserial -> serial port object
%                : text     -> text to output to the VN-100     
% Output(s)      : None
%**************************************************************************
%   Examples:
%   
%   Set the VN-100 to data output rate of 100Hz with ADOR = YPR.
%
%   s = VNserial('COM8');   
%   VNprintf(VNserial, 'VNWRG,07,100');
%   VNprintf(VNserial, 'VNWRG,06,1');
%

% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function VNprintf(VNserial, text)

s = sprintf('$%s*%s\n', text, VNchecksum(text));
fprintf(VNserial, s);

end