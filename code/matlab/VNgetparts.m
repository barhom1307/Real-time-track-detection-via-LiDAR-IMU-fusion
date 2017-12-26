%**************************************************************************
% Function Name  : VNgetparts
% Description    : 
%  VNgetparts divides the given string up into sections based upon the
%   comma delimiter. The leading $ and trailing * will be removed from the
%   input string prior to dividing it into parts.
%
% Input(s)       : str   -> string to divide based upon delimiter    
% Output(s)      : parts -> array of string parts
%**************************************************************************
%   Examples:
%
%   Get the parts to the following input command.
%
%   s = '$VNWRG,06,255,0*72';
%   VNgetparts(s);
%
%   This will divide s into the following parts
%   ans = {'VNWRG'    '06'    '255'    '0', '72'}

% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function parts = VNgetparts(str)

%Replace any occurances of * with ,
str = strrep(str, '*', ',');

%Remove any occurances of the $ character.
str = strrep(str, '$', '');

rem = str;
i = 1;

while length(findstr(rem, ',')) > 0
    [s, rem] = strtok(rem, ',');
    parts{i} = s;
    i = i + 1;
end