%**************************************************************************
% Function Name  : VNregID
% Description    : 
%  Converts a register 3 to 5 letter acronym into a register number. This
%  allows for the user to easily requests a known type of data without
%  having to remember the associated register number.
%
% Input(s)       : register acronym
% Output(s)      : regID -> register ID
%**************************************************************************
% Examples:
%
%   s = VNserial('COM8');
%
%   regID = VNregID('QTN');
%
%   vals = VNreadregister(s, regID);
%
% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function regID = VNregID(regName)


if strcmp(regName, 'MODEL')
    regID = 1;
elseif strcmp(regName, 'HWREV')
    regID = 2;
elseif strcmp(regName, 'SN')
    regID = 3;
elseif strcmp(regName, 'FWVER')
    regID = 4;
elseif strcmp(regName, 'BAUD')
    regID = 5;
elseif strcmp(regName, 'ADOR')
    regID = 6;
elseif strcmp(regName, 'ADOF')
    regID = 7;
elseif strcmp(regName, 'YPR')
    regID = 8;
elseif strcmp(regName, 'QTN')
    regID = 9;
elseif strcmp(regName, 'QMR')
    regID = 15;
elseif strcmp(regName, 'MAG')
    regID = 17;
elseif strcmp(regName, 'ACC')
    regID = 18;
elseif strcmp(regName, 'GYR')
    regID = 19;
elseif strcmp(regName, 'MAR')
    regID = 20;
elseif strcmp(regName, 'REF')
    regID = 21;
elseif strcmp(regName, 'HSI')
    regID = 23;
elseif strcmp(regName, 'ACT')
    regID = 25;
elseif strcmp(regName, 'RFR')
    regID = 26;
elseif strcmp(regName, 'YMR')
    regID = 27;
elseif strcmp(regName, 'IMU')
    regID = 54;
elseif strcmp(regName, 'GPS')
    regID = 58;
elseif strcmp(regName, 'GPE')
    regID = 59;
elseif strcmp(regName, 'INS')
    regID = 63;
elseif strcmp(regName, 'INE')
    regID = 64;
elseif strcmp(regName, 'ISL')
    regID = 72;
elseif strcmp(regName, 'ISE')
    regID = 73;
elseif strcmp(regName, 'DTV')
    regID = 80;
elseif strcmp(regName, 'YBA')
    regID = 239;
elseif strcmp(regName, 'YIA')
    regID = 240;
end