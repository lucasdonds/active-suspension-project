%% Lucas' serial function
function port = startMacSerial()

port = serial('/dev/cu.usbmodem141401'); % to find ports, either check in 
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