%% Close port function
function port = closePort(portName)
    %'/dev/cu.usbmodem143401'
    
    port = serial(portName);
    instrfind;
    fclose(ans);
    instrfind
end