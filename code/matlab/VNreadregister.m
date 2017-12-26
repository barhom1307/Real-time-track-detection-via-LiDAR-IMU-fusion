%**************************************************************************
% Function Name  : VNreadregister
% Description    : 
%  Get the contents of the requested register from the VN-100. The register
%  can be given as a register ID number or as a 3-5 letter acronym found in
%  in the VNregID function.
%
% Input(s)       : s        -> serial port object
%                : regID    -> register ID or acronym
%                : option1  -> first optional parameter
%                : option2 - > second optional parameter
% Output(s)      : response -> The contents of the requested register as a
%                              1XN array.
%**************************************************************************
% Examples:
%
%   s = VNserial('COM8');
%
%   %Read quaternion register
%   vals = VNreadregister(s, 9); 
%   
%   % Also you could use a register acronym:
%   vals = VNreadregister(s, 'QTN');
%
% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function response = VNreadregister(VNserial, regID, option1, option2)

%Convert regID to reg ID number if string
if isstr(regID)
    regID = VNregID(regID);
end

%Create command string
if nargin < 3
    s = sprintf('VNRRG,%i', regID);
elseif nargin < 4
    s = sprintf('VNRRG,%i,%i', regID, option1);
else
    s = sprintf('VNRRG,%i,%i,%i', regID, option1, option2);
end

%Clear input buffer
VNclearbuffer(VNserial);

%Write command to device
VNprintf(VNserial, s);

%Get response from device
resp = VNverifyresponse(VNserial, s);

%Get parts of response
parts = VNgetparts(resp);

if(strcmp(parts{1}, 'VNERR'))
    fprintf('Error: %s\n', VNerrormsg(str2double(parts{2})));
    response = [];
    return;
end

%Check register ID
regID = str2double(parts{2});

switch regID
    case 0 %Tag
        response = char(parts{3});
    case 1 %Model
        response = char(parts{3});
    case 2 %HwRev
        response = uint8(str2double(parts{3}));
    case 3 %SN
        response = char(parts{3});
    case 4 %FwVer
        response = char(parts{3});
    %All other
    otherwise
        %Determine number of parameters in register
        numParam = length(parts) - 3;
        
        response = zeros(1, numParam);
        
        for i=1:numParam
            response(i) = str2double(parts{2+i});
        end
end