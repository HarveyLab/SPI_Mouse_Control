function receive_velocity
    % Define the serial port object
    % Adjust 'COM3' to your specific port
    serialPort = 'COM4'; 
    baudRate = 9600; % Set baud rate to match Teensy's configuration
    
    dt = 10;
    tau = 20;
    alpha1 = dt./(dt + tau);
    alpha = 1-exp(-dt./tau);
    
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
        fprintf(s,'000000');
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
    
    t = D(:,2);
    p = D(:,3);
    r = D(:,4);
    y = D(:,5);
    v = D(:,[3:5]);
    
    alpha = 1;
    v_smooth = zeros(size(v));
    for n = 2:numDataPoints
        v_smooth(n,:) = alpha * v(n,:) + (1 - alpha) * v(n-1,:);
    end
    
    figure;hold on;
    plot(t,v_smooth(:,1),'b-');
    plot(t,v_smooth(:,2),'r-');
    plot(t,v_smooth(:,3),'g-');
    
    v_area = trapz(t, v) * 1e-6
    
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
