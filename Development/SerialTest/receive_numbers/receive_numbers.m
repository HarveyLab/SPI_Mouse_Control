% Define the serial port object
% Adjust 'COM3' to your specific port
serialPort = 'COM4'; 
baudRate = 9600; % Set baud rate to match Teensy's configuration

delete(instrfind);
s = serial('COM4');
set(s,'BaudRate',57600);
s.Terminator = 'CR/LF';
fopen(s);

% Initialize a variable to store the received data
D = [];

% Set the number of data points to read (adjust as needed)
numDataPoints = 1e3;

% Read data from the serial port
for i = 1:numDataPoints
    flushinput(s);
    % fprintf(s,'000000');
    d = str2double(fgetl(s))
    D = [D;d];
end

% Close the serial port
fclose(s);
delete(s);
clear s;

% Display the received data
disp('Received Data:');
disp(D);

% Plot the received data (optional)
figure;
plot(D);
xlabel('Sample Number');
ylabel('Data Value');
title('Data Received Over Serial Port');
