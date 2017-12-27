clear ;
clc;

%creating velocity vector from 1km/h to 20km/h (0.2 intervals)
velocity_kmh = 1:0.2:20;
velocity = velocity_kmh*0.277777778; % changing the vector into m/sec

time_const = 0.1 ;%in sec

%calc distance vector
distance = time_const*velocity;

%ploting graphs
figure
plot(velocity_kmh,distance,'g')
title('Distance as function of cars velocity when time = 0.1sec')
xlabel('Car velocity (km/h)')
ylabel('Distance (m)')
% finding the point's number at 10kmh
index1 = find(velocity_kmh==10);
y1 = distance(index1);
str = ['\uparrow 10km/h = ',num2str(y1)];
text(9.9,y1-0.01,str,'HorizontalAlignment','left');
% finding the point's number at 15kmh
index2 = find(velocity_kmh==15);
y2 = distance(index2);
str = ['\uparrow 15km/h = ',num2str(y2)];
text(14.9,y2-0.01,str,'HorizontalAlignment','left');
