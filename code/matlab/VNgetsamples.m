%**************************************************************************
% Function Name  : VNgetsamples
% Description    : 
%  VNgetsamples gets the given number of samples from a requested register.
%  The user can select which register and how many samples they want.  The
%  data will be returned in a NxM array where N is the number of samples
%  and M is the number of parameters for the requested register.
%
% Input(s)       : s          -> serial port object
%                : register   -> register acronym
%                : numSamples -> number of samples to capture
% Output(s)      : data       -> samples requested as NxM matrix
%**************************************************************************
%   Examples:
%
%   s = VNserial('COM8');
%
%   data = VNgetsamples(s, 'YPR', 100);
%   plot(data);
%

% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function data = VNgetsamples(s, register, numSamples)

%Convert regID to reg ID number if string
if isstr(register)
    register = VNregID(register);
end

%Get sample data
sample = VNreadregister(s, register);

%Zero out return matrix
data = zeros(numSamples, size(sample,2));

%Get requested data as array
for i=1:numSamples
    data(i,:) = VNreadregister(s, register);
end