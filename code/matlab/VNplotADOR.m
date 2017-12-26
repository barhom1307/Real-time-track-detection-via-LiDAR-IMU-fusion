%**************************************************************************
% Function Name  : VNplotADOR
% Description    : 
%  Plot the data captured by the VNrecordADOR function. Pass the data
%  received along with the ADOR register type, and this function will plot
%  the results along with labeled and titled figures.
%
% Input(s)       : ADOR       -> register acronym
%                : sampleRate -> sample rate in Hz
%                : numSec     -> number of seconds to record
%                : titleName  -> Prefix for plot titles (optional)
% Output(s)      : None
%**************************************************************************
% Examples:
%
%   s = VNserial('COM8');
%
%   data = VNrecordADOR(s, 'YPR', 100, 10); %record @ 100Hz for 10s
%
%   VNplotADOR(data, 'YPR', 100);
%
% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function VNplotADOR(data, ADOR, sampleRate, titleName)

if(isstr(ADOR))
    ADOR = VNgetADORID(ADOR);
end

if nargin < 4
    titleName = '';
end

%Plot results
if ADOR == 1 %YPR
    plotYPR(data, sampleRate, titleName);
elseif ADOR == 2 %QTN
    plotQuat(data, sampleRate, titleName);   
elseif ADOR == 8 %QMR
    plotQuat(data(:,1:4), sampleRate, titleName);
    plotMagnetic(data(:,5:7), sampleRate, titleName);    
    plotAccel(data(:,8:10), sampleRate, titleName);
    plotGyro(data(:,11:13), sampleRate, titleName); 
elseif ADOR == 10 %MAG
    plotMagnetic(data, sampleRate, titleName); 
elseif ADOR == 11 %ACC
    plotAccel(data, sampleRate, titleName);
elseif ADOR == 12 %'GYR'
    plotGyro(data, sampleRate, titleName); 
elseif ADOR == 13 %'MAR'
    plotMagnetic(data(:,1:3), sampleRate, titleName); 
    plotAccel(data(:,4:6), sampleRate, titleName);
    plotGyro(data(:,7:9), sampleRate, titleName);    
elseif ADOR == 14 %'YMR'
    plotYPR(data(:,1:3), sampleRate, titleName);
    plotMagnetic(data(:,4:6), sampleRate, titleName); 
    plotAccel(data(:,7:9), sampleRate, titleName);
    plotGyro(data(:,10:12), sampleRate, titleName); 
elseif ADOR == 16 %'YBA'
    plotYPR(data(:,1:3), sampleRate, titleName);
    plotMagnetic(data(:,4:6), sampleRate, titleName); 
    plotAccel(data(:,7:9), sampleRate, titleName);
    plotGyro(data(:,10:12), sampleRate, titleName);
elseif ADOR == 17 %'YIA'
    plotYPR(data(:,1:3), sampleRate, titleName);
    plotMagnetic(data(:,4:6), sampleRate, titleName); 
    plotAccel(data(:,7:9), sampleRate, titleName);
    plotGyro(data(:,10:12), sampleRate, titleName);
elseif ADOR == 19 %'IMU'
    plotMagnetic(data(:,1:3), sampleRate, titleName); 
    plotAccel(data(:,4:6), sampleRate, titleName);
    plotGyro(data(:,7:9), sampleRate, titleName);
    plotTemp(data(:,10), sampleRate, titleName);
    plotPres(data(:,11), sampleRate, titleName);  
end
end

function plotYPR(data, sampleRate, titleName)
    figure;
    t = (1:size(data,1))./sampleRate;
    stitle = sprintf('%s Yaw / Pitch / Roll at %i Hz', titleName, sampleRate); 
    plot(t,data); title(stitle);
    xlabel('Time [s]'); ylabel('Angle [deg]');
    legend('Yaw', 'Pitch', 'Roll');
end

function plotQuat(data, sampleRate, titleName)
    figure;
    t = (1:size(data,1))./sampleRate;
    stitle = sprintf('%s Quaterion at %i Hz', titleName, sampleRate); 
    plot(t,data); title(stitle);
    xlabel('Time [s]'); ylabel('quaternion component');
    legend('Q_0', 'Q_1', 'Q_2', 'Q_3');
end

function plotMagnetic(data, sampleRate, titleName)
    figure;
    t = (1:size(data,1))./sampleRate;
    stitle = sprintf('%s Magnetic at %i Hz', titleName, sampleRate); 
    plot(t,data); title(stitle);
    xlabel('Time [s]'); ylabel('Magnetic [custom units]');
    legend('X', 'Y', 'Z');
end

function plotAccel(data, sampleRate, titleName)
    figure;
    t = (1:size(data,1))./sampleRate;
    stitle = sprintf('%s Acceleration at %i Hz', titleName, sampleRate); 
    plot(t,data); title(stitle);
    xlabel('Time [s]'); ylabel('Acceleration [m/s^2]');
    legend('X', 'Y', 'Z');    
end

function plotGyro(data, sampleRate, titleName)
    figure;
    t = (1:size(data,1))./sampleRate;
    stitle = sprintf('%s Angular Rates at %i Hz', titleName, sampleRate); 
    plot(t,data.*(180/pi)); title(stitle);
    xlabel('Time [s]'); ylabel('Angular Rates [deg/s]');
    legend('X', 'Y', 'Z');    
end

function plotTemp(data, sampleRate, titleName)
    figure;
    t = (1:size(data,1))./sampleRate;
    stitle = sprintf('%s Temperature at %i Hz', titleName, sampleRate); 
    plot(t,data); title(stitle);
    xlabel('Time [s]'); ylabel('Temperature [C]');
end

function plotPres(data, sampleRate, titleName)
    figure;
    t = (1:size(data,1))./sampleRate;
    stitle = sprintf('%s Pressure at %i Hz', titleName, sampleRate); 
    plot(t,data); title(stitle);
    xlabel('Time [s]'); ylabel('Pressure [kPa]');
end
