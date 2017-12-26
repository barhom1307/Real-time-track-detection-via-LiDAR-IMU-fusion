%**************************************************************************
% Function Name  : VNwriteregister
% Description    : 
%  Write to the contents of the given register on the VN-100. The register
%  can be given as a register ID number or as a 3-5 letter acronym found in
%  in the VNregID function.
%
% Input(s)       : VNserial -> serial port object
%                : regID    -> register ID or acronym
%                : values   -> values to write to the register
%                : optional -> optional parameters      
% Output(s)      : response -> The contents of the requested register as a
%                              1XN array.
%**************************************************************************
%   Examples:
%   
%   Set the Hard/Soft Iron compensation register (HSI) to the default state
%   of an identity matrix for soft iron and zero for hard iron.
%
%   s = serial('COM8', 'BaudRate', 115200, 'DataBits', 8);
%   fopen(VNserial);
%   
%   %Write to register 23
%   VNwriteRegister(s, 23, [1 0 0 0 1 0 0 0 1 0 0 0]);
%
%   %Alternatively you can use the register acronym
%   VNwriteRegister(s, 'HSI', [1 0 0 0 1 0 0 0 1 0 0 0]);

% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function response = VNwriteregister(VNserial, regID, values, optional)

%Convert regID to reg ID number if string
if isstr(regID)
    regID = VNregID(regID);
end

%Create one string for entire command
if ischar(values)
    if nargin < 4
        s = sprintf('VNWRG,%i,%s', regID, values);
    else
        s = sprintf('VNWRG,%i,%s,%i', regID, values, optional);
    end
else
    params = '';
    for i=1:length(values)
        params = [params sprintf(',%1.9G', values(i))];
    end
    if nargin >= 4
        s = sprintf('VNWRG,%i%s,%i', regID, params, optional);
    else
        s = sprintf('VNWRG,%i%s', regID, params);
    end
end

%Clear input buffer
VNclearbuffer(VNserial);

%Send command
VNprintf(VNserial, s);

%Wait for response
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