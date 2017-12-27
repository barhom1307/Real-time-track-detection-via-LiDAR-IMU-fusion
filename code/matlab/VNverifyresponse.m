%**************************************************************************
% Function Name  : VNverifyresponse
% Description    : 
%  VNverifyresponse waits for the response to return from the given sent
%   command. This function will lock up the serial port until a response is
%   received from the issued command. If a response is not received in the
%   first attempt then the function will exit and return an empty array. 
%   If a response is received and the type of message returned is similar
%   to the message sent then the function will return the received message.
%   The function does not check to ensure that the values returned from
%   the device match the values sent.
%
% Input(s)       : VNserial -> serial port object
%                : s        -> command to await response for  
% Output(s)      : response -> the response returned by the device
%**************************************************************************
%   Examples:
%
%   Send a command to change the data output rate to 100Hz and then wait
%   for the response.
%
%   VNserial = serial('COM8', 'BaudRate', 115200, 'DataBits', 8);
%   fopen(VNserial);
%   
%   s = 'VNWRG,07,100';
%   VNprintf(VNserial, s);
%   VNverifyresponse(VNserial, s);
%

% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function response = VNverifyresponse(VNserial, s)

%Get response from serial port
try
    response = fgets(VNserial);
catch err
    %Display error to user
    Disp(err)
    
    %Return 0
    response = [];
    return
end

%Get parts of response
partsResponse = VNgetparts(response);

%Get parts of sent command
partsCommand = VNgetparts(s);

%Check to see if first part of response is equal to first part of sent
%command
if (strcmp(partsResponse{1}, partsCommand{1}) ~= 1 && strcmp(partsResponse{1}, 'VNERR') ~= 1)
    response = [];
end