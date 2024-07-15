function receive_velocity
    % Define the serial port object
    % Adjust 'COM3' to your specific port
    serialPort = 'COM4'; 
    baudRate = 9600; % Set baud rate to match Teensy's configuration
    
    

    % Initialize a variable to store the received data
    D = [];
    delete(instrfind);
    
    s = serial('COM4');
    set(s,'BaudRate',57600);
    s.Terminator = 'CR/LF';
    fopen(s);

    % Define the cleanup task
    cleanupObj = onCleanup(@() cleanupFunction(s));

    % Set the number of data points to read (adjust as needed)
    numDataPoints = 1e3;

    % Read data from the serial port
    while size(D,1) <= numDataPoints
        % flushinput(s);
        % fprintf(s,'000000');
        k = 1;
        data = [];
        while s.BytesAvailable>0
            for k = 1:7
                temp = fgetl(s);
                val_str = strsplit(temp,' = ');
                if k==1 && ~strcmp(val_str{1},'Counter')
                    break
                end
                data(k) = str2double(val_str{2});
            end
            data
            D = [D;data];
            flushinput(s);
        end
    end

    % D
    figure;
    stem(diff(D(:,1)))
    
    figure;
    plot(D(:,3:5))
    
    return

    % Plot the received data (optional)
    figure;
    plot(D);
    xlabel('Sample Number');
    ylabel('Data Value');
    title('Data Received Over Serial Port');
end

function cleanupFunction(s)
    % Define the cleanup task to be executed on script termination
    disp('Executing cleanup tasks...');
    % Close the serial port
    fclose(s);
    delete(s);
    clear s;   
end
