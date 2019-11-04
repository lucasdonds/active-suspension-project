%% Close port function
function port = closePort()
    instrfind;
    fclose(ans);
    instrfind
end