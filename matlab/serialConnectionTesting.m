%% Active Suspension serial connection testing Software
% written by Lucas Dondertman Fall 2019
%correspondin arduino software is called "test_for_matlab_connection"
%%
close all; clear all; clc;

%s=startSerial();    % initialize serial connection for windows i guess
s=startMacSerial();  % initialize serial connection for mac


for rows = 1:1
    for columns = 1:1
    readData=fscanf(s,'%d',[1,1]);     %might work after fixing fprintf and fopen
    testArray(rows, columns)= readData(1,1);
    end
end
testArray
fclose(s); 
delete(s);
clear s;

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

port = serial('/dev/cu.usbmodem141301'); % to find ports, either check in 
%Arduino IDE and pick the same as the arduino, or go to terminal and type
%  -> ls /dev/cu.*  <-  then pick same one as arduino
fclose(port);
    try 
        set(port, 'BaudRate', 115200);
        set(port, 'OutputBufferSize', 7);
        configureTerminator(port,"CR");
        port.Terminator
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

%%
