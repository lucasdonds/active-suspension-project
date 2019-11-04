%% Active Suspension Demonstration Test Software
% written by Rachel Du Spring 2019, Edited by Eugene Li Fall 2019,
% Edited by Lucas Dondertman Fall 2019
%% Main
close all; clear all; clc;

%s=startSerial();    % initialize serial connection for windows i guess
s=startMacSerial('/dev/cu.usbmodem141401');  % initialize serial connection for mac  

numRows=700;
numCols=12;

testArray = zeros(numRows,numCols);
for rows = 1:numRows
    for columns = 1:numCols
    readData=fscanf(s,'%f',[1,1]);    
    testArray(rows, columns)= readData(1,1);
    end
end

millis = testArray(:,1);  %first col
carPosition= testArray(:,2);   %second col, etc
wheelPosition = testArray(:,3);
roadPosition = testArray(:,4);
motorSpeed = testArray(:,5);
averageSpeed = testArray(:,6);
xAccel = testArray(:,7);
yAccel = testArray(:,8);
zAccel = testArray(:,9);
xAccelTop = testArray(:,10);
yAccelTop = testArray(:,11);
zAccelTop = testArray(:,12);

figure; %plot acceleration of mid plate from mpu9250
plot(millis, xAccel, millis, yAccel, millis, zAccel);
title('Acceleration of Mid'); 

figure; %plot acceleration of top plate from mpu9250
plot(millis, xAccelTop, millis, yAccelTop, millis, zAccelTop);
title('Acceleration of Top'); 

figure; %plot positions vs time of the three plates
plot(millis, carPosition, millis, wheelPosition, millis, roadPosition);
title('Positions');
xlabel('Time (ms)');
ylabel('Position (m)');
legend('Car (top)', 'Wheel (middle)', 'Road (bottom)') ;

figure; %plot motorspeed and average motor speed
plot(millis, motorSpeed, millis, averageSpeed);
title('Road Actuation Speed');
xlabel('Time (ms)');
ylabel('Speed (rpm)');
legend('Current', 'Average');
closePort();
clear s;

%% run this section only if arduino cannot upload because port is busy or 
% there is a fail to open port error 
closePort();
