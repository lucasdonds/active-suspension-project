function port = resetMotor(portName)

    port = serial(portName);
        set(port, 'BaudRate', 115200);
        set(port, 'OutputBufferSize', 512);
        set(port,'DataBits',8);
        set(port,'StopBits',1);
        set(port,'Parity','none');
        set(port, 'Terminator', 'CR/LF');

    c = 'd';
    while (c~='c') 
        c=strtrim(fscanf(port));
    end
    if (c=='c')
        disp('Serial read success');
    end
    fprintf(port,'%c','c');
    disp('Motor Reset');
end