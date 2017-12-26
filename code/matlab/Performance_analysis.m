clear ;
clc;

%creating distance vector from 1m to 30m (0.5m intervals)
distance = 1:0.5:30;

%defining cones length
cone_length_bottom = 0.175;
cone_length_middle = 0.125;
cone_length_up = 0.055;

%creating functions
points_on_cone_bottom = 10*(atand(cone_length_bottom./(2*distance)));
points_on_cone_middle = 10*(atand(cone_length_middle./(2*distance)));
points_on_cone_up = 10*(atand(cone_length_up./(2*distance)));

%ploting graphs
figure

% bottom_area_of_cone graph
ax1 = subplot(3,1,1); 
plot(ax1,distance,points_on_cone_bottom,'g')
title(ax1,'Number of points on bottom area')
xlabel('Distance from Lidar(m)')
ylabel('Number of points')
% finding the point's number at 5m
index1 = find(distance==5);
y1 = points_on_cone_bottom (index1);
str = ['\downarrow',num2str(y1)];
text(4.9,y1+6,str,'HorizontalAlignment','left');
% finding the point's number at 10m
index2 = find(distance==10);
y2 = points_on_cone_bottom (index2);
str = ['\downarrow',num2str(y2)];
text(9.9,y2+6,str,'HorizontalAlignment','left');

% middle_area_of_cone graph
ax2 = subplot(3,1,2); 
plot(ax2,distance,points_on_cone_middle,'r')
title(ax2,'Number of points on middle area')
xlabel('Distance from Lidar(m)')
ylabel('Number of points')
% finding the point's number at 5m
index3 = find(distance==5);
y1 = points_on_cone_middle (index3);
str = ['\downarrow',num2str(y1)];
text(4.9,y1+4,str,'HorizontalAlignment','left');
% finding the point's number at 10m
index4 = find(distance==10);
y2 = points_on_cone_middle (index4);
str = ['\downarrow',num2str(y2)];
text(9.9,y2+4,str,'HorizontalAlignment','left');

% up_area_of_cone graph
ax3 = subplot(3,1,3); 
plot(ax3,distance,points_on_cone_up,'b')
title(ax3,'Number of points on up area')
xlabel('Distance from Lidar(m)')
ylabel('Number of points')
% finding the point's number at 5m
index5 = find(distance==5);
y1 = points_on_cone_up (index5);
str = ['\downarrow',num2str(y1)];
text(4.9,y1+3,str,'HorizontalAlignment','left');
% finding the point's number at 10m
index6 = find(distance==10);
y2 = points_on_cone_up (index6);
str = ['\downarrow',num2str(y2)];
text(9.9,y2+3,str,'HorizontalAlignment','left');




