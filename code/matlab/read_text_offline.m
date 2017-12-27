clear;
clc;

file_name='C:\text_Lidar\data_file_2.txt';

%openning the data file, and place it's data in a matrix:
fileID=fopen(file_name,'r');
formatSpec='%f %f %f';
size=[3,inf];
%Read the file data, filling output array XYZ, in column order. 
%formatSpec- mentioning it's in float format, size- defining the size of matrix- 3 rows, infinte columns.
XYZ=fscanf(fileID,formatSpec,size);
fclose(fileID);

%preforming transpose in order to get the format of: [x y z]:
XYZ=XYZ.'; 

%filtering the data:
%indexes=find(XYZ(:,3)>0 & XYZ(:,3)<1 & abs(XYZ(:,2))<10 & abs(XYZ(:,1))<10);
%indexes=find(XYZ(:,3>-0.2 & XYZ(:,3)<1.5 & abs(XYZ(:,1))<10 & abs(XYZ(:,2))<10);
indexes=XYZ(:,3)>-0.4 & XYZ(:,3)<2 & abs(XYZ(:,1))<5 & abs(XYZ(:,2))<5 & (XYZ(:,1)~=0 | XYZ(:,2)~=0 |XYZ(:,3)~=0);
%indexes=XYZ(:,3)>-0.1 & XYZ(:,3)<5 & abs(XYZ(:,1))<5 & abs(XYZ(:,2))<5 & (XYZ(:,1)~=0 | XYZ(:,2)~=0 |XYZ(:,3)~=0);

temp(:,1)=XYZ(indexes,1);
temp(:,2)=XYZ(indexes,2);
temp(:,3)=XYZ(indexes,3);
XYZ=temp;

M=ones(length(XYZ(:,1)),3)*64;
Intensity=uint8(M);

%transforming the x,y,z points in to point cloud format:
ptCloud= pointCloud(XYZ(:,1:3),'Color',Intensity);
pcshow(ptCloud);

