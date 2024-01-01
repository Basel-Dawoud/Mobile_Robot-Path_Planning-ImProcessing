% Data Acquisition & Serial Communication with Arduino

clc;
clear;

% Define the serial port and baud rate
serialPort = "COM10"; % Change this to the appropriate port
baudRate = 9600;

% Create a serial object
s = serialport(serialPort, baudRate);
pause(5);

% Read and display data from Arduino
data = [];
errorFlag = false;

for i = 1:180
    % Read and convert data from the serial port
    try
        newData = str2double(readline(s));
        data = [data; newData];
    catch
        errorFlag = true;
        break;
    end
end

if ~errorFlag
    disp(data);
else
    disp('Error: Unable to read data from Arduino.');
end
