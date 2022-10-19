%%Main thesis code written by Luke Su and Stanley Xu, 1/06/2020
clc; clear all; close all
%Read file and store as table
%filename = 'sample_ydomain';
filename = 'walk2';
rawData = table2array(readtable(filename));

%Seperate data into right and left leg 9-axis data and clear rawData
[left,right] = allocateData(rawData);
clear rawData;
%end of function.

[world] = worldcoordinates(left,right);
%Getting the locations of gait cycles
[left_locations, right_locations, left_y_velocity, x_velocity, y_velocity, left_x_velocity, left_velocity,right_x_velocity, new_world] = integrating(world, left, right);

%% Plotting
close all

figure 
plot(world.left.x)
hold on 
plot(world.right.x)
xlabel('time')
ylabel('Meters/s^2')
title('Normalized Acceleration')

figure
plot(-left.angle.y)
hold on
plot(right.angle.y)
xlabel('time')
ylabel('Degrees')
title('Angular Displacement')

figure
plot(left_x_velocity)
hold on
plot(right_x_velocity)
xlabel('Gait Cycles')
ylabel('Meters/second')
title('Velocity in the x direction')
%% Functions

function [left,right] = allocateData(rawData)
%this function will convert raw data into 2 arrays, left will contain
%data from the left leg and right will contain data from the right leg
data_size = length(rawData);

%Append array so that last variable is acceleration
for i = 0:data_size-2
   if rawData(data_size-i,1) == 81 && rawData(data_size-i-1,1) == 85
      rawData = rawData(1:data_size-i-2,:);
      break
   end
end

%Split matrix into 1-D arrays (For analysis purposes)
rightData = rawData(:,1); leftData = rawData(:,2); time = rawData(:,3);
%Declare structures
%Left structure
left.acceleration.x = []; left.acceleration.y = []; left.acceleration.z = []; left.acceleration.time = [];
left.angularvelocity.x = []; left.angularvelocity.y = []; left.angularvelocity.z = []; left.angularvelocity.time = [];
left.angle.x = []; left.angle.y = []; left.angle.z = []; left.angle.time = [];
%Right Structure
right.acceleration.x = []; right.acceleration.y = []; right.acceleration.z = []; right.acceleration.time = [];
right.angularvelocity.x = []; right.angularvelocity.y = []; right.angularvelocity.z = []; right.angularvelocity.time = [];
right.angle.x = []; right.angle.y = []; right.angle.z = []; right.angle.time = [];


n = length(rightData);

for i = 1:11:n
    left_temp = leftData(i:i+10);
    right_temp = rightData(i:i+10);
    time_temp = time(i:i+10);

    
    %Raw IMU data to 9 axis data with time
    [left_data,left_mode] = decodeIMU(left_temp);
    [right_data,right_mode] = decodeIMU(right_temp);
    
    switch left_mode
        case 1
            left.acceleration.x = [left.acceleration.x,left_data(1)];
            left.acceleration.y = [left.acceleration.y,left_data(2)];
            left.acceleration.z = [left.acceleration.z,left_data(3)];
            left.acceleration.time = [left.acceleration.time,time_temp(2)];
        case 2 
            left.angularvelocity.x = [left.angularvelocity.x,left_data(1)];
            left.angularvelocity.y = [left.angularvelocity.y,left_data(2)];
            left.angularvelocity.z = [left.angularvelocity.z,left_data(3)];
            left.angularvelocity.time = [left.angularvelocity.time,time_temp(2)];
        case 3
            left.angle.x = [left.angle.x,left_data(1)];
            left.angle.y = [left.angle.y,left_data(2)];
            left.angle.z = [left.angle.z,left_data(3)];
            left.angle.time = [left.angle.time,time_temp(2)];      
    end
    
    switch right_mode
        case 1
            right.acceleration.x = [right.acceleration.x,right_data(1)];
            right.acceleration.y = [right.acceleration.y,right_data(2)];
            right.acceleration.z = [right.acceleration.z,right_data(3)];
            right.acceleration.time = [right.acceleration.time,time_temp(2)];
        case 2 
            right.angularvelocity.x = [right.angularvelocity.x,right_data(1)];
            right.angularvelocity.y = [right.angularvelocity.y,right_data(2)];
            right.angularvelocity.z = [right.angularvelocity.z,right_data(3)];
            right.angularvelocity.time = [right.angularvelocity.time,time_temp(2)];
        case 3
            right.angle.x = [right.angle.x,right_data(1)];
            right.angle.y = [right.angle.y,right_data(2)];
            right.angle.z = [right.angle.z,right_data(3)];
            right.angle.time = [right.angle.time,time_temp(2)];      
    end    
           
end


end 


function [data,mode] = decodeIMU(array) 
%check temp array and seperate it into variables
%mode 1 = acceleration, mode 2 = angular velocity, mode 3 = angle
    data = zeros(1,3);
    mode = 0;
    LS8 = 256; %Left shift 8, equivalent to mulitplication by 256.
    g = 9.8; %gravitational acceleration 

    switch array(2)
        case 81
        data(1) = g*check_sign_16(bitor(array(4)*LS8,array(3)))/32768.0*16;
        data(2) = g*check_sign_16(bitor(array(6)*LS8,array(5)))/32768.0*16;
        data(3) = g*check_sign_16(bitor(array(8)*LS8,array(7)))/32768.0*16;;
        mode = 1;
        case 82
        data(1) = check_sign_16(bitor(array(4)*LS8,array(3)))/32768.0*2000;
        data(2) = check_sign_16(bitor(array(6)*LS8,array(5)))/32768.0*2000;
        data(3) = check_sign_16(bitor(array(8)*LS8,array(7)))/32768.0*2000; 
        mode = 2;
        case 83
        data(1) = check_sign_16(bitor(array(4)*LS8,array(3)))/32768.0*180;
        data(2) = check_sign_16(bitor(array(6)*LS8,array(5)))/32768.0*180;
        data(3) = check_sign_16(bitor(array(8)*LS8,array(7)))/32768.0*180;
        mode = 3;
    end
end 


function signed_int = check_sign_16(uint) 
%This function converts an unsigned 16 bit number to it's signed equivalent
   if uint >= 32768 
       signed_int = uint - 65536;
   else 
       signed_int = uint;
   end
end

function [world] = worldcoordinates(left,right)
    %Declare structure for storing world co-ordinates.
    world.left.x = []; world.left.y = [];
    world.right.x = []; world.right.y = [];
    %Determine theeta for right and left leg
    theta_right = (right.angle.y)*pi/180; theta_left = (left.angle.y)*pi/180;
    gravity = 9.8;
    %Left leg world co-ordinates
    world.left.x = abs(left.acceleration.x.*cos(theta_left)) - abs(left.acceleration.y.*sin(theta_left));
    world.left.y = abs(left.acceleration.x.*sin(theta_left)) + abs(left.acceleration.y.*cos(theta_left)) - gravity;
    %Right leg world co-ordinates
    world.right.x = abs(right.acceleration.x.*cos(theta_right)) - abs(right.acceleration.y.*sin(theta_right));
    world.right.y = abs(right.acceleration.x.*sin(theta_right)) + abs(right.acceleration.y.*cos(theta_right)) - gravity;
end

function [left_locations, right_locations, left_y_velocity, x_velocity, y_velocity, left_x_velocity, left_velocity, right_x_velocity, new_world] = integrating(world, left, right)

   %Left leg gait cycle seperation
   j = 1;
   left.angle.y = -left.angle.y;
   for i = 1:length(left.angle.y)-1
        if  left.angle.y(i) <0 &&  left.angle.y(i+1) > 0
            if abs(left.angle.y(i)) < abs(left.angle.y(i+1))
                left_locations(j) = i;
                j = j+1;
            else
                    left_locations(j) = i+1;
                    j = j+1;
            end 
        end 
    end
   
   % Gait cycle separation with the right leg
   j = 1;
   for i = 1:length(right.angle.y)-1
        if  right.angle.y(i) <0 &&  right.angle.y(i+1) > 0
            if abs(right.angle.y(i)) < abs(right.angle.y(i+1))
                right_locations(j) = i;
                j = j+1;
            else
                    right_locations(j) = i+1;
                    j = j+1;
            end 
        end 
    end
    
    %Half rectifying acceleration data
    for i = 1:length(world.left.x)-1
        if world.left.x(i) <0
            new_world(i) = 0;
            i = i+1;
        else
            new_world(i) = world.left.x(i);
            i = i+1;
        end
    end
    
    for i = 1:length(world.left.x)-1
        if world.left.x(i) <0
            world.left.x(i) = 0;
            i = i+1;
        else
            i = i+1;
        end
    end
    
   for i = 1:length(world.right.x)-1
        if world.right.x(i) <0
            world.right.x(i) = 0;
            i = i+1;
        else
            i = i+1;
        end
   end
    
    %Left Leg Velocity
    for i = 1:length(left_locations) - 1
        if i == 1
            left_y_velocity(i) = trapz(left.acceleration.time(1:left_locations(i)), world.left.y(1:left_locations(i)));
            left_x_velocity(i) = trapz(left.acceleration.time(1:left_locations(i)), world.left.x(1:left_locations(i)));
            left_velocity(i) = sqrt(left_x_velocity(i)^2+left_y_velocity(i)^2);
            i = i+1;
        else 
            left_y_velocity(i) = trapz(left.acceleration.time(left_locations(i):left_locations(i+1)), world.left.y(left_locations(i):left_locations(i+1)));
            left_x_velocity(i) = trapz(left.acceleration.time(left_locations(i):left_locations(i+1)), world.left.x(left_locations(i):left_locations(i+1)));
            left_velocity(i) = sqrt(left_x_velocity(i)^2+left_y_velocity(i)^2);
            i = i+1;
        end 
    end

    %Right Leg Velocity
    for i = 1:length(right_locations) - 1
        if i == 1
            right_y_velocity(i) = trapz(right.acceleration.time(1:right_locations(i)), world.right.y(1:right_locations(i)));
            right_x_velocity(i) = trapz(right.acceleration.time(1:right_locations(i)), world.right.x(1:right_locations(i)));
            right_velocity(i) = sqrt(right_x_velocity(i)^2+right_y_velocity(i)^2);
            i = i+1;
        else 
            right_y_velocity(i) = trapz(right.acceleration.time(right_locations(i):right_locations(i+1)), world.right.y(right_locations(i):right_locations(i+1)));
            right_x_velocity(i) = trapz(right.acceleration.time(right_locations(i):right_locations(i+1)), world.right.x(right_locations(i):right_locations(i+1)));
            right_velocity(i) = sqrt(right_x_velocity(i)^2+right_y_velocity(i)^2);
            i = i+1;
        end 
    end
    x_velocity = (mean(left_x_velocity)+mean(right_x_velocity))/2;
    y_velocity = (mean(left_y_velocity)+mean(right_y_velocity))/2;
end


