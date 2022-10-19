%% Code for testing direct intergration method with 2 IMU Sensors
%Written by Luke Su - 14/06/2020
close all; clc; clear all;

%Delcare file
filename = '';
combined_array = table2array(readtable(filename)); 

%Note: Combined array has all array elements, 1-10 leg 1, 11-20 leg 2
%seperate full array into legs 1 and 2
array_l1 = combined_array(:,1:10);
array_l2 = combined_array(:,11:20);

%Split elements into x and y.
[acceleration_l1, angularv_l1,angle_l1,time_l1] = splitarray(array_l1);
[acceleration_l2, angularv_l2,angle_l2,time_l2] = splitarray(array_l2);
%Determine World co-ordinates for both legs
[l1_xworld,l1_yworld] = WorldCoordinates(array_l1);
[l2_xworld,l2_yworld] = WorldCoordinates(array_l2);

plot(l2_xworld);
hold on
%plot(acceleration_l1(:,1));
title('leg1_x');
figure
plot(l2_yworld);
hold on
%plot(acceleration_l1(:,2));
title('leg1_y');

function [x,y] = WorldCoordinates(array)
    %initialize variables.
    x_acceleration = (array(:,1)); y_acceleration = (array(:,2));
    x = zeros(1,length(array));y = zeros(1,length(array));
    theeta = (array(:,8))*pi/180; gravity = 9.8;
    
    %Determine World co-ordinates
    for i = 1:length(array)
       x(i) = x_acceleration(i)*cos(theeta(i)) - y_acceleration(i)*sin(theeta(i));
       y(i) = x_acceleration(i)*sin(theeta(i)) + y_acceleration(i)*cos(theeta(i)) - gravity;
    end
end 

function [acceleration,angular_v,angle,timer] = splitarray(array)
   acceleration = array(:,1:3);
   angular_v = array(:,4:6);
   angle = array(:,7:9);
   timer = array(:,10);
end


