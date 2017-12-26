%****************************************************
%Acquiring GPS information X-Y-Z and Yaw, Pitch, Roll
%Written on 4/12/17 by Aviv
%Register 59 are used to read location information.
% X,Y,Z (0,0,0 being the Earth's center)
%Register 63 is used to recieve Yaw,Pitch, and Roll information.
% 5000 samples per minute, input is in minutes.
%*****************************************************
function [Loc_X_Y_Z, YPR] = Get_Loc_and_YPR(stop)
tic
Loc_X_Y_Z = zeros(stop,3);
YPR = zeros(stop,3);

for i=1:stop
    %Acquire location and save to matrix:
    temp = VNreadregister(VNserial('COM3'),59);
    Loc_X_Y_Z(i,:) = temp(5:7);
    %Acquire Yaw, Pitch, Roll and save to matrix:
    temp = VNreadregister(VNserial('COM3'),63);
    YPR(i,:) = temp(5:7);
end
toc