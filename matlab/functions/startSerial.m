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