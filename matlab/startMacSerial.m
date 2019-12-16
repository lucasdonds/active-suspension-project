%% Lucas' serial function
function port = startMacSerial(portName)

port = serial(portName); % to find ports, either check in 
%Arduino IDE and pick the same as the arduino, or go to terminal and type
%  -> ls /dev/cu.*  <-  then pick same one as arduino
fclose(port);
    try 
        set(port, 'BaudRate', 115200);
        set(port, 'OutputBufferSize', 512);
        set(port, 'InputBufferSize', 768);
        set(port,'DataBits',8);
        set(port,'StopBits',1);
        set(port,'Parity','none');
        set(port, 'Terminator', 'CR/LF');
        set(port, 'Timeout', 20);
        fopen(port);
    catch
        instrfind;
        fclose(ans);
        instrfind;
        fopen(port);
    end
    instrfind

end