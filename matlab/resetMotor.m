function port = resetMotor(portName)
    a = 'b';
    while (a~='s') 
        a=strtrim(fscanf(portName));
    end
    disp('start motor reset');
    while (a~='e')
        a=strtrim(fscanf(portName));
    end
    if (a=='e')
        disp('finished motor reset');
    end
end