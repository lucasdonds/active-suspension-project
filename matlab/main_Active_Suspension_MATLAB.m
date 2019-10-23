%% Active Suspension Demonstration Test Software
% written by Rachel Du Spring 2019, Edited by Eugene Li Fall 2019,
% Edited by Lucas Dondertman Fall 2019
%% Main
close all; clear all; clc;

%s=startSerial();    % initialize serial connection for windows i guess
s=startMacSerial();  % initialize serial connection for mac

%prompt = 'Enter speed between 0 and 90 rpm: ';
%x = input(prompt);
%speed = num2str(x);
%s
%fprintf(s, speed);   

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
fclose(s); 
delete(s);
clear s;

%% run this section only if arduino cannot upload because port is busy or 
% there is a fail to open port error 
closePort();

%% Serial functions 
function msp = startSerial()
% find out which port Arduino is connected to:
portNum=2;
wrongPort=1;
while wrongPort
   msp = serial(['COM' num2str(portNum)]);
    try 
        set(msp, 'BaudRate', 115200);
        set(msp, 'OutputBufferSize', 7);
        fopen(msp); %go to catch if can't open
        wrongPort=0; % found the right one - exit while loop!
    catch 
        delete(msp);
        clear msp
        instrreset % close any wrongly opened connection
        wrongPort =1 ; % keep trying... 
    end
    portNum=portNum+1;
      if portNum==16
          error('Arduino is not connected')
      end
end
end

%/dev/cu.usbmodem141301

%% Lucas' serial function
function port = startMacSerial()

port = serial('/dev/cu.usbmodem143401'); % to find ports, either check in 
%Arduino IDE and pick the same as the arduino, or go to terminal and type
%  -> ls /dev/cu.*  <-  then pick same one as arduino
fclose(port);
    try 
        set(port, 'BaudRate', 115200);
        set(port, 'OutputBufferSize', 7);
        fopen(port);
    catch
        mbox = msgbox('open failed!'); uiwait(mbox);
        instrfind
        fclose(ans);
        instrfind
        fopen(port);
        instrfind
    end
    
end


%% If port is stuck open, use this function to close it

function port = closePort()

port = serial('/dev/cu.usbmodem141401');
instrfind;
fclose(ans);
instrfind
end


