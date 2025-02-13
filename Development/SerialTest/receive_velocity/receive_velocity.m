function receive_velocity
    % Define the serial port object
    % Adjust 'COM3' to your specific port
    % serialPort = 'COM4'; 
    % baudRate = 9600; % Set baud rate to match Teensy's configuration
    
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
    numDataPoints = 1000;

    % Read data from the serial port
    while size(D,1) <= numDataPoints
        % if s.BytesAvailable>0
            flushinput(s);
            fprintf(s,'000000');
            % write(s,"000000","string");
            d = parseTeensyMessage(fgetl(s));
            % mvData = [];
            d_vec = [d.dp d.dr d.dy d.l1 d.l2 d.v1 d.v2 d.dta d.dtmsg]
            D = [D;d_vec];
            % mvData = [D;d.dp, d.dr, d.dy];
            % flushinput(s);
            size(D,1)
        % end
    end

    % D
    figure;
    stem(D(2:end,9))
    
    % t = D(:,2);
    t = cumsum(D(:,9)) - D(1,9);
    p = D(:,1);
    r = D(:,2);
    y = D(:,3);
    v = D(:,1:3);
    
    % alpha = 1;
    v_smooth = zeros(size(v));
    tau = 50; % [ms]
    for n = 2:numDataPoints
        dt = D(n,9);
        alpha = 1 - exp(-dt / tau);
        v_smooth(n,:) = alpha * v(n,:) + (1 - alpha) * v(n-1,:);
    end
    
    figure;hold on;
    y_lim = 500;
    subplot(3,1,1);
    plot(t,v_smooth(:,1),'b-');
    ylim([-y_lim y_lim])
    subplot(3,1,2);
    plot(t,v_smooth(:,2),'r-');
    ylim([-y_lim y_lim])
    subplot(3,1,3);
    plot(t,v_smooth(:,3),'g-');
    ylim([-y_lim y_lim])
    
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
