%% Active Suspension Demonstration Test Software
% written by Rachel Du Spring 2019, Edited by Eugene Li Fall 2019,
% Edited by Lucas Dondertman Fall 2019
%% Main
close all; clear all; clc;

%s=startSerial();    % initialize serial connection for windows i guess
s=startMacSerial('/dev/cu.usbmodem141401');  % initialize serial connection for mac  

%resetMotor(s);

numRows=100;
numCols=15;

testArray = zeros(numRows,numCols);
for rows = 1:numRows
    for columns = 1:numCols
    readData=fscanf(s,'%f',[1,1]);    
    testArray(rows, columns)= readData(1,1);
    end
end

seconds = testArray(:,1)./1000;       %first col, etc... divide by 1000 to change ms to s
carPosition = testArray(:,2)./1000;   %(Zs) divide col by 1000 to change mm to m
wheelPosition = testArray(:,3)./1000; %(Zus) divide col by 1000 to change mm to m
roadPosition = testArray(:,4)./1000;  %divide by col 1000 to change mm to m
motorSpeed = testArray(:,5);
averageSpeed = testArray(:,6);
xAccel = testArray(:,7);
yAccel = testArray(:,8);
zAccel = testArray(:,9);
xAccelTop = testArray(:,10);
yAccelTop = testArray(:,11);
zAccelTop = testArray(:,12);
carVelocity = testArray(:,13);
wheelVelocity = testArray(:,14);
roadVelocity = testArray(:,15);

figure; %plot acceleration of mid plate from mpu9250
plot(seconds, xAccel, seconds, yAccel, seconds, zAccel);
title('Acceleration of Mid'); 
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Z (vertical)','Y','X');

figure; %plot acceleration of top plate from mpu9250
plot(seconds, xAccelTop, seconds, yAccelTop, seconds, zAccelTop);
title('Acceleration of Top'); 
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Z (vertical)','Y','X');

figure; %plot positions vs time of the three plates
plot(seconds, carPosition, seconds, wheelPosition, seconds, roadPosition);
title('Positions');
xlabel('Time (s)');
ylabel('Position (m)');
legend('Car (top)', 'Wheel (middle)', 'Road (bottom)') ;

% figure; %plot motorspeed and average motor speed
% plot(seconds, motorSpeed, seconds, averageSpeed);
% title('Road Actuation Speed');
% xlabel('Time (s)');
% ylabel('Speed (rpm)');
% legend('Current', 'Average');

figure; %plot velocities of plates 
plot(seconds, carVelocity, seconds, wheelVelocity, seconds, roadVelocity);
title('Plate Velocities');
xlabel('Time (s)');
ylabel('Speed (m/s)');
legend('car', 'wheel', 'road')

closePort();
clear s;

%% run this section only if arduino cannot upload because port is busy or 
% there is a fail to open port error 
closePort();
