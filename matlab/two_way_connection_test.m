%% Main Code Section
clear all; close all; clc;
s = startMacSerial('/dev/cu.usbmodem141401');

x=fscanf(s);  %Read from arduino
x=strip(x);   %Remove new line from string (remove terminator)
x

if x=="Start Serial Communication"              %read array of numbers from arduino
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
fwrite(s,testArray(1,5));     %write number (5) from array to arduino, will wait for arduino to reviece
y=fscanf(s);                  %recieve from arduino again
y = strtrim(y)                %remove trailing whitespace from terminator

if y=='6'                     %outputs yes if it all worked
   output = 'yes'
else
    output = 'no'
end
 
closePort('/dev/cu.usbmodem141401');      %close port at end

%% Run this section only if there is a problem with port
closePort('/dev/cu.usbmodem141401');
