%**************************************************************************
% Function Name  : VNchecksum
% Description    : 
%  VNchecksum calculates the checksum for the given serial string.  The
%  checksum is calculated for all of the characters in a command excluding
%  the $ and the * characters.
%
% Input(s)       : str      -> the text to calculate the checksum on  
% Output(s)      : CS       -> The calculated checksum
%**************************************************************************
%   Examples:
%   
%   Determine the checksum for the command to write 1 to register 6.
%
%   s = 'VNWRG,06,1';
%   ck = Checksum(s);
%
%   In this particular case the correct checksum calculated for
%   'VNWRG,06,1' should be:
%   ck = '6D'

% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function CS = VNchecksum(str)

p = zeros(1,1,'uint8');

for i=1:size(str,2)
    p = bitxor(p,cast(str(i),'uint8'));
end

CS = dec2hex(p);

if length(CS) == 1
    CS = ['0' CS];
end

end