%**************************************************************************
% Function Name  : VNrecordADOR
% Description    : 
%  Records measurements from the device using the ADOR (asyncronous data
%  output register). This function can be used to record a batch of data
%  from the device at high speed.  Before calling this function you will
%  want to set the baud rate high enough to ensure that the device can
%  ouptut the data at the requested rate.  The data will be returned in a
%  [m, n] array where m is the number of samples taken and n is the number
%  of elements each message for the requested ADOR output type.
%
% Input(s)       : s          -> serial output object
%                : ADOR       -> async data output register type 
%                                (see below lists for possible options)
%                : sampleRate -> The sample rate in Hz.
%                : numSec     -> Number of seconds to record data.
% Output(s)      : data       -> recorded data as MxN array
%**************************************************************************
% Examples:
%
%   s = VNserial('COM1');
%   data = VNrecordADOR(s, 'YPR', 100, 10);
%   plot(data);
%
% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function data = VNrecordADOR(s, ADOR, sampleRate, numSec)

%Set the data output rate
VNwriteregister(s, 7, sampleRate);

%Set the ADOR
if strcmp(ADOR, 'YPR')
    VNwriteregister(s, 6, 1);
    data = zeros(sampleRate*numSec, 3);
elseif strcmp(ADOR, 'QTN')
    VNwriteregister(s, 6, 2);
    data = zeros(sampleRate*numSec, 4);
elseif strcmp(ADOR, 'QMR')
    VNwriteregister(s, 6, 8);
    data = zeros(sampleRate*numSec, 13);
elseif strcmp(ADOR, 'MAG')
    VNwriteregister(s, 6, 10);
    data = zeros(sampleRate*numSec, 3);
elseif strcmp(ADOR, 'ACC')
    VNwriteregister(s, 6, 11);
    data = zeros(sampleRate*numSec, 3);
elseif strcmp(ADOR, 'GYR')
    VNwriteregister(s, 6, 12);
    data = zeros(sampleRate*numSec, 3);
elseif strcmp(ADOR, 'MAR')
    VNwriteregister(s, 6, 13);
    data = zeros(sampleRate*numSec, 9);
elseif strcmp(ADOR, 'YMR')
    VNwriteregister(s, 6, 14);
    data = zeros(sampleRate*numSec, 12);
elseif strcmp(ADOR, 'YBA')
    VNwriteregister(s, 6, 16);
    data = zeros(sampleRate*numSec, 12);
elseif strcmp(ADOR, 'YIA')
    VNwriteregister(s, 6, 17);
    data = zeros(sampleRate*numSec, 12);
elseif strcmp(ADOR, 'IMU')
    VNwriteregister(s, 6, 19);
    data = zeros(sampleRate*numSec, 11);
elseif strcmp(ADOR, 'GPS')
    VNwriteregister(s, 6, 20);
    data = zeros(sampleRate*numSec, 15);
elseif strcmp(ADOR, 'GPE')
    VNwriteregister(s, 6, 21);
    data = zeros(sampleRate*numSec, 15);
elseif strcmp(ADOR, 'INS')
    VNwriteregister(s, 6, 22);
    data = zeros(sampleRate*numSec, 15);
elseif strcmp(ADOR, 'INE')
    VNwriteregister(s, 6, 23);
    data = zeros(sampleRate*numSec, 15);
elseif strcmp(ADOR, 'ISL')
    VNwriteregister(s, 6, 28);
    data = zeros(sampleRate*numSec, 15);
elseif strcmp(ADOR, 'ISE')
    VNwriteregister(s, 6, 29);
    data = zeros(sampleRate*numSec, 15);
elseif strcmp(ADOR, 'DTV')
    VNwriteregister(s, 6, 30);
    data = zeros(sampleRate*numSec, 7);
end

%Create parse string
ps = '%*6c';
for i=1:size(data,2)
    ps = [ps ',%g'];
end

%Record data
fprintf('Data recording started.\n');
for i=1:numSec
    for j=1:sampleRate
        data((i-1)*sampleRate+j, :) = fscanf(s, ps);
    end
    
    fprintf('%i  seconds remaining...\n', numSec-i);
end

%Turn off ADOR
VNprintf(s, 'VNWRG,6,0');
pause(0.1);
VNclearbuffer(s);

fprintf('Data recording complete.\n');
