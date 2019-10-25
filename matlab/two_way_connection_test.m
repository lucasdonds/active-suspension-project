%% Main Code Section
clear all; close all; clc;
s = startMacSerial('/dev/cu.usbmodem141401');
% s=serial('/dev/cu.usbmodem141401','BaudRate',115200);
% set(s, 'Terminator', 'LF');
% fopen(s);

x=fscanf(s);  %Read 
x=strip(x);   %Remove new line from string (remove terminator)
x

if x=="Start Serial Communication"
    numRows=5;
    numCols=5;
    
    testArray = zeros(numRows,numCols);
    for rows = 1:numRows
        for columns = 1:numCols
        readData=fscanf(s,'%f',[1,1]);    
        testArray(rows, columns)= readData(1,1);
        end
    end
    testArray
end
fwrite(s,testArray(1,5));
y=fscanf(s);
y
 
% x=0;
% x=fscanf(s);
% fwrite(s,y);
% y=fscanf(s);
% x
% y
closePort('/dev/cu.usbmodem143401');

%% Run this section only if there is a problem with port
closePort('/dev/cu.usbmodem143401');
