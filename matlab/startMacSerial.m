%% Lucas' serial function
function port = startMacSerial(portName)

port = serial(portName); % to find ports, either check in 
%Arduino IDE and pick the same as the arduino, or go to terminal and type
%  -> ls /dev/cu.*  <-  then pick same one as arduino
fclose(port);
    try 
        set(port, 'BaudRate', 115200);
        set(port, 'OutputBufferSize', 512);
        set(port,'DataBits',8);
        set(port,'StopBits',1);
        set(port,'Parity','none');
        set(port, 'Terminator', 'CR/LF');
        fopen(port);
    catch
        instrfind;
        fclose(ans);
        instrfind;
        fopen(port);
    end
    instrfind
    a = 'b';
    while (a~='a') 
        a=strtrim(fscanf(port));
    end
    if (a=='a')
        disp('Serial read success');
    end
    fprintf(port,'%c','a');
    mbox = msgbox('Serial Communication setup'); uiwait(mbox);
end