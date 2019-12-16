%% Active Suspension Demonstration Test Software
% written by Rachel Du Spring 2019, Edited by Eugene Li Fall 2019,
% Edited by Lucas Dondertman Fall 2019
%% Main
clear all; clc;

%s=startSerial();    % initialize serial connection for windows
s=startMacSerial('/dev/cu.usbmodem141401');  % initialize serial connection for mac  
    
% Serial Handshake
a = 'b';
while (a~='a') 
    a=strtrim(fscanf(s));
end
if (a=='a')
    disp('Handshake read success');
end
fprintf(s,'%c','a');
disp('Serial communication set up');

% Motor reset handshake
c = 'd';
while (c~='c') 
    c=strtrim(fscanf(s));
end
if (c=='c')
    disp('Motor reset read success');
end
fprintf(s,'%c','c');
disp('Motor Reset + Setpoint found');

% Size of array for data collection, only change rows, cols refers to
% quantities (eg: carVelocity)
numRows=500;
numCols=10;

disp('Starting read');
testArray = zeros(numRows,numCols);
for rows = 1:numRows
    for columns = 1:numCols
        readData = fscanf(s,'%f',[1,1]);    
        testArray(rows, columns)= readData(1,1);
    end
end
disp('All values read');

testArray;

seconds = testArray(:,1)./1000;       %first col, etc... divide by 1000 to change ms to s
carPosition = testArray(:,2)./1000;   %divide col by 1000 to change mm to m
wheelPosition = testArray(:,3)./1000; %divide col by 1000 to change mm to m
roadPosition = testArray(:,4)./1000;  %divide by col 1000 to change mm to m
xAccelTop = testArray(:,5);
yAccelTop = testArray(:,6);
zAccelTop = testArray(:,7);
carVelocity = testArray(:,8);
wheelVelocity = testArray(:,9);
roadVelocity = testArray(:,10);

figure; %plot acceleration of top plate from mpu9250
plot(seconds, xAccelTop, seconds, yAccelTop, seconds, zAccelTop);
title('Acceleration of Top'); 
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Z (vertical)','Y','X');
% 
figure; %plot positions vs time of the three plates
plot(seconds, carPosition, seconds, wheelPosition, seconds, roadPosition);
title('Positions');
xlabel('Time (s)');
ylabel('Position (m)');
legend('Car (top)', 'Wheel (middle)', 'Road (bottom)') ;

figure; %plot velocities of plates 
plot(seconds, carVelocity, seconds, wheelVelocity, seconds, roadVelocity);
title('Plate Velocities');
xlabel('Time (s)');
ylabel('Speed (m/s)');
legend('car', 'wheel', 'road')

closePort();

%clear s;

%% run this section only if arduino cannot upload because port is busy or 
% there is a fail to open port error 
closePort();
