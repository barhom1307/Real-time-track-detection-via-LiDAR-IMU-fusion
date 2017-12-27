%**************************************************************************
% Function Name  : VNimportDocklight
% Description    : 
%  Import data from docklight into matlab. Setup the VN-100 to output data
%  at the desired rate asyncronously using the ADOR register. Then in
%  doclight use Use Tools->Start comm. logging and choose ASCII. Click both
%  boxes in the advanced settings area.  Click play to start datalogging,
%  and stop then Tools->Stop comm. logging to end data logging.  Then call
%  this function along with the name of the text file to import into
%  matlab.
%
% Input(s)       : filename -> filename of text file to import
% Output(s)      : data     -> data as NxM array where M is the number of
%                              items in the requested register.
%**************************************************************************
% Examples:
%
%   data = importDocklight('C:\test1_asc.txt');
%
% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function data = VNimportDocklight(filename)

f = fopen(filename);

% Drop first line since it is possibly not complete or empty
fgets(f);

%Grab second line
line = fgets(f);

%Now find number of parts
numParts = length(findstr(',',line));

%Build lookup string
searchString = '%*s';
for i=1:numParts
    searchString = [searchString ' %f'];
end
searchString = [searchString '%*2c'];

fclose(f);
f = fopen(filename);

%Remove first two lines
line = fgets(f);
line = fgets(f);

%Get data
C = textscan(f, searchString, 'Delimiter', [',' '*'], 'CollectOutput', 1);

%Convert cell to matrix
data = cell2mat(C);

%Close file
fclose(f);

%Drop last line since it may not be complete
C = C(1:end-1,:);