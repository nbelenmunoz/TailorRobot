%% COMPLETE MATLAB CODE 
close all; clear; clc;

%% SELECT WHICH TOOLS TO USE
simulo = 0;          % 0 = real hardware, 1 = simulation only
if simulo == 1    
    addr = [1,1];
end

robot1 = 1; 
estrusore = 1;

toolvar = estrusore;
toolvec = robot1;
if estrusore == 1
    toolvec = [toolvec, toolvec(end) + estrusore];
end

%% CONNECTIONS (check IP and COM)
clc;
if simulo == 0
    tout = 20; conntout = 30;
    
    if robot1 == 1
        ip_robot = "192.168.0.22";   % change to your robot's IP
        port_robot = 10008;
        t_robot = ADDRESS(ip_robot, port_robot, tout, conntout);
        configureTerminator(t_robot, "CR");
        fprintf("Robot connected at %s:%d\n", ip_robot, port_robot);
    else
        t_robot = [];
    end

    if estrusore == 1
        arduinoPort = "COM6";        % change to your Arduino port
        baudRate = 9600;
        try
            t_arduino = serialport(arduinoPort, baudRate);
            configureTerminator(t_arduino, "LF");
            fprintf("Arduino connected on %s at %d baud\n", arduinoPort, baudRate);
        catch ME
            error("Could not open %s: %s", arduinoPort, ME.message);
        end
    else
        t_arduino = [];
    end
    
    if (robot1 && isempty(t_robot)) || (estrusore && isempty(t_arduino))
        error("Connection failed.");
    end
end

%% POINTS - pure horizontal movement at Z=240, robot speed = 2 mm/s
% Stitch length = 10 mm -> motor speed = (600/10)*2 = 120 pps (36 RPM) -> high torque
M = [
    230.00,  15.00,  260.00,  -180.00,  0.00,  -90.00,  2,  0,  1;   % P1: approach
    230.00,  15.00,  240.00,  -180.00,  0.00,  -90.00,  2,  0,  0;   % P2: vertical down (no sewing)
    340.00,  15.00,  240.00,  -180.00,  0.00,  -90.00,  2,  1,  0;   % P3: horizontal (sewing on segment P2->P3)
    340.00,  15.00,  260.00,  -180.00,  0.00,  -90.00,  2,  0,  1;   % P4: vertical up (no sewing)
];

Points = M;
% plotPath(Points, 0);   % optional

%% EXTRACT DATA
nbuff = 1;
PPnum = Points(:,1:6);
vel_prog = Points(:,7);
vel_prog(1) = 2;          % ensure approach speed is also 2 mm/s
extrudeFlag = Points(:,8);
delaytime = Points(:,end);
np = size(PPnum,1);

PPnum = round(PPnum,2);
vel_prog = round(vel_prog,2);
delaytime = round(delaytime,2);

% Convert positions to robot string format
for i = 1:np
    flag = "(6,0)";
    PPstr(i,1) = genP(PPnum(i,:), flag);
end

%% SEWING PARAMETERS (max torque)
L_puntada = 10;               % stitch length [mm]
STEPS_PER_STITCH_REV = 600;   % steps per stitch
MAX_SPS = 3000;               % safety limit (actual is much lower)
MIN_SPS = 10;

RAMP_FRAC = 0.15;             % fraction of travel time for ramps
RAMP_DT = 0.05;               % time step between ramp commands [s]
RAMP_MAX_T = 0.30;            % max ramp duration [s]

%% PREPARE ARDUINO (manual positioning)
if estrusore && simulo == 0
    fprintf('\n=== MOTOR PREPARATION ===\n');
    writeline(t_arduino, 'D');
    fprintf('Motor DISABLED. Position needle manually (e.g., needle up).\n');
    input('Press ENTER when ready...', 's');
    writeline(t_arduino, 'E');
    fprintf('Motor ENABLED.\n');
    pause(0.5);
    writeline(t_arduino, 'S0');
    pause(0.1);
    fprintf('Ready.\n\n');
end

%% TEACH ROBOT FIRST POINTS
if robot1 && simulo == 0
    fprintf('Teaching robot (np=%d, nbuff=%d)...\n', np, nbuff);
    writeline(t_robot, num2str(np));
    pause(0.2);
    writeline(t_robot, num2str(nbuff));
    pause(1.0);
    for k = 1:nbuff
        write(t_robot, PPstr(k,1));
        pause(0.5);
        write(t_robot, num2str(vel_prog(k)));
        pause(1.0);
    end
    pause(0.5);
    writeline(t_robot, '0');            % move to first point
    handshake = "P-fatto";
    temp = CONTROL(t_robot, handshake); % wait for "PRONTO"
    fprintf('Robot: %s\n', temp);
end

%% MAIN LOOP
data = strings(np,1);
tempovec = zeros(np,2);

% Start process: send '1'
if robot1 && simulo == 0
    writeline(t_robot, '1');
    data(1) = CONTROL(t_robot, handshake);   % arrival at point 2
    tempovec(1,1) = posixtime(datetime('now'));
    fprintf("P-1 reached (point 2)\n");
end

% Loop over points 2 to np-1
for k = 2 : np-1
    % Send next point and speed (for point k+1) if not last
    if k < np
        if robot1 && simulo == 0
            fprintf('k=%d | TX PP: %s\n', k, PPstr(k+1,1));
            writeline(t_robot, PPstr(k+1,1));
            fprintf('k=%d | TX VEL: %s\n', k, num2str(vel_prog(k+1)));
            writeline(t_robot, num2str(vel_prog(k+1)));
        end
    end
    
    % Wait for arrival at current point k
    if robot1 && simulo == 0
        data(k) = CONTROL(t_robot, handshake);
        tempovec(k,1) = posixtime(datetime('now'));
    end
    fprintf("P-%d reached\n", k);
    
    % Sewing on segment from k to k+1?
    if estrusore && extrudeFlag(k) == 1 && k < np
        p_curr = PPnum(k,1:3);
        p_next = PPnum(k+1,1:3);
        dist = norm(p_next - p_curr);
        vel_robot = vel_prog(k);
        
        if dist > 0 && vel_robot > 0
            travel_time = dist / vel_robot;   % seconds
            sps = (STEPS_PER_STITCH_REV / L_puntada) * vel_robot;
            sps = max(MIN_SPS, min(MAX_SPS, sps));
            
            t_ramp = min(travel_time * RAMP_FRAC, RAMP_MAX_T);
            t_cruise = travel_time - 2*t_ramp;
            if t_cruise < 0
                t_cruise = 0;
                t_ramp = travel_time / 2;
            end
            
            fprintf('  Segment P%d→P%d: dist=%.1f mm, vel=%.1f mm/s, t=%.3f s, motor=%.0f pps\n', ...
                    k, k+1, dist, vel_robot, travel_time, sps);
            
            % Send CONTINUE to robot (start moving)
            writeline(t_robot, '1');
            pause(0.05);
            
            % Acceleration ramp
            ramparMotor(t_arduino, 0, sps, t_ramp, RAMP_DT);
            
            % Cruise
            if t_cruise > 0
                writeline(t_arduino, sprintf('S%.0f', sps));
                pause(t_cruise);
            end
            
            % Deceleration ramp
            ramparMotor(t_arduino, sps, 0, t_ramp, RAMP_DT);
            writeline(t_arduino, 'S0');
            
            fprintf('  Sewing completed.\n');
            tempovec(k,2) = posixtime(datetime('now'));
        else
            fprintf('  Zero distance/velocity, skipping sewing.\n');
            writeline(t_robot, '1');
        end
    else
        % No sewing: just send CONT and optional delay
        if robot1 && simulo == 0
            pause(delaytime(k));
            writeline(t_robot, '1');
        end
    end
end

%% FINISH
if estrusore && simulo == 0
    writeline(t_arduino, 'S0');
    pause(0.05);
    writeline(t_arduino, 'S0');
    writeline(t_arduino, 'D');
    fprintf('Motor stopped and disabled.\n');
end
fprintf("PROCESS FINISHED\n");

%% HOME
if robot1 && simulo == 0
    strHome = "PARTO";
    temp = MOVE(t_robot, '1', strHome);
    fprintf("Robot returning home...\n");
end

rawdata = data;
save("rawdata.mat", "rawdata");

%% LOCAL FUNCTION
function ramparMotor(t_arduino, v_start, v_end, t_ramp, dt)
    if t_ramp <= 0 || abs(v_end - v_start) < 2
        if v_end <= 0
            writeline(t_arduino, 'S0');
        else
            writeline(t_arduino, sprintf('S%.0f', v_end));
        end
        return;
    end
    n = max(2, round(t_ramp / dt));
    speeds = round(linspace(v_start, v_end, n));
    for i = 1:n
        if speeds(i) <= 0
            writeline(t_arduino, 'S0');
        else
            writeline(t_arduino, sprintf('S%.0f', speeds(i)));
        end
        pause(dt);
    end
end
