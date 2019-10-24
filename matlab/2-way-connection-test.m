
clear all; close all; clc;
s=serial('/dev/cu.usbmodem143401','BaudRate',115200);
fopen(s);
y=2;
x=0;
x=fscanf(s);
fwrite(s,y);
y=fscanf(s);
x
y
closePort();

%%
closePort();
%% If port is stuck open, use this function to close it

function port = closePort()

port = serial('/dev/cu.usbmodem143401');
instrfind;
fclose(ans);
instrfind
end