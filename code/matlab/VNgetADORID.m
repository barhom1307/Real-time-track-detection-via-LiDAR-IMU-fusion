%**************************************************************************
% Function Name  : VNgetADORID
% Description    : 
%  Converts a ADOR register 3 to 5 letter acronym into a register number. 
%  This allows for the user to easily requests a known type of data without
%  having to remember the associated register number.
%
% Input(s)       : ADOR     -> register acronym
% Output(s)      : regID    -> register ID
%**************************************************************************
% Examples:
%
%   s = VNserial('COM8');
%
%   regID = VNgetADORID('QTN');
%
%   VNwriteregister(s, 'ADOR' regID);
%
% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function regID = VNgetADORID(ADOR)

regID = 1;

%Set the ADOR
if strcmp(ADOR, 'YPR')
    regID = 1;
elseif strcmp(ADOR, 'QTN')
    regID = 2;
elseif strcmp(ADOR, 'QMR')
    regID = 8;
elseif strcmp(ADOR, 'MAG')
    regID = 10;
elseif strcmp(ADOR, 'ACC')
    regID = 11;
elseif strcmp(ADOR, 'GYR')
    regID = 12;
elseif strcmp(ADOR, 'MAR')
    regID = 13;
elseif strcmp(ADOR, 'YMR')
    regID = 14;
elseif strcmp(ADOR, 'YBA')
    regID = 16;
elseif strcmp(ADOR, 'YIA')
    regID = 17;
elseif strcmp(ADOR, 'IMU')
    regID = 19;
elseif strcmp(ADOR, 'GPS')
    regID = 20;
elseif strcmp(ADOR, 'GPE')
    regID = 21;
elseif strcmp(ADOR, 'INS')
    regID = 22;
elseif strcmp(ADOR, 'INE')
    regID = 23;
elseif strcmp(ADOR, 'ISL')
    regID = 28;
elseif strcmp(ADOR, 'ISE')
    regID = 29;
elseif strcmp(ADOR, 'DTV')
    regID = 30;
end