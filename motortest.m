close all; 
clear all;
clc;

%% EXTRUDER TEST CONFIGURATION
simulo = 0;  % Set to 1 for simulation, 0 for real connection
robot1 = 0;   % Disable robots
robot2 = 0;   % Disable robots  
estrusore = 1; % Enable extruder
plc = 0;      % Disable PLC

%% CONNECTION SETUP
if simulo == 0
    % Extruder connection parameters
    addr = "192.168.0.6";  % Extruder IP address
    port = 1000;           % Extruder port
    tout = 20;             % Timeout
    conntout = 30;         % Connection timeout
    
    % Create connection
    t_extruder = ADDRESS(addr, port, tout, conntout);
    fprintf('Extruder connection created\n');
end

%% EXTRUDER INITIALIZATION
if estrusore == 1 
    temperatura = 0;       % Temperature in Celsius
    omega = (pi/4);        % Angular velocity
    extrvel = (10^6)/(4*254.77*omega);  % Extruder velocity calculation
    tempospurgo = 2*pi/omega;           % Purge time
    gspurgo = extrvel;                  % Purge velocity
    
    % Start extruder with purge cycle
    data = EXTRSTART(t_extruder, temperatura, tempospurgo, gspurgo);
    fprintf('Extruder initialized and purge cycle started\n');
    
    % Wait for purge to complete
    pause(tempospurgo + 2);  % Add 2 seconds safety margin
    fprintf('Purge cycle completed\n');
end

%% TEST EXTRUDER WITH DIFFERENT SPEEDS
if estrusore == 1
    fprintf('Testing extruder with different speeds...\n');
    
    % Test different extruder speeds
    test_speeds = [300, 200, 150, 100, 50, 10, 0];  % Speed values to test
    
    for i = 1:length(test_speeds)
        speed = test_speeds(i);
        command = sprintf("G%dn", speed);
        
        fprintf('Sending command: %s\n', command);
        write(t_extruder, command);
        
        % Wait between speed changes
        if i < length(test_speeds)
            pause(3);  % Wait 3 seconds between speed changes
        end
    end
    
    fprintf('Extruder speed test completed\n');
end

%% EXTRUDER STOP
if estrusore == 1
    % Stop extruder
    write(t_extruder, "G0n");
    pause(0.02);
    write(t_extruder, "G0n");  % Repeat for safety
    
    fprintf('Extruder stopped\n');
    
    % Close connection
    if simulo == 0
        clear t_extruder;
        fprintf('Extruder connection closed\n');
    end
end

fprintf('Extruder test completed successfully!\n');
