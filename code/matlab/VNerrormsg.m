
function msg = VNerrormsg(code)

switch code
    case 1
        msg = 'Hard Fault';
    case 2
        msg = 'Serial Buffer Overflow';
    case 3
        msg = 'Invalid Checksum';
    case 4
        msg = 'Invalid Command';
    case 5
        msg = 'Not Enough Parameters';
    case 6
        msg = 'Too Many Parameters';
    case 7
        msg = 'Invalid Parameter';
    case 8
        msg = 'Invalid Register';
    case 9
        msg = 'Unauthorized Access';
    case 10
        msg = 'Watchdog Reset';
    case 11
        msg = 'Output Buffer Overflow';
    case 12
        msg = 'Insufficient Baud Rate';
    case 255
        msg = 'Error Buffer Overflow';
end

end