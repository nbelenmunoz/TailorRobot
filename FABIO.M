close all; 
clear; 
clc;    
%%
%%% SELECT WHICH TOOLS TO USE
simulo = 0;
if simulo == 1    
    addr = [1,1];
end
robot1 = 1; estrusore = 1;
toolvar = estrusore;
toolvec = robot1;
if estrusore == 1
    toolvec = [toolvec, toolvec(end) + estrusore];
end

%%% CONNECTIONS
clc;
if simulo == 0
    tout = 20; conntout = 30;
    if robot1 == 1
        ip_robot = "192.168.0.22";  
        %ip_robot = "127.0.0.1";
        port_robot = 10008;
        t_robot = ADDRESS(ip_robot, port_robot, tout, conntout);
        configureTerminator(t_robot, "CR");
        fprintf("Robot connected at %s:%d\n", ip_robot, port_robot);
    else
        t_robot = [];
    end
    if estrusore == 1
        arduinoPort = "COM6";
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

%%% POINTS MATRIX (robot speed = 5 mm/s, stitch length = 10 mm)
% M = X, Y, Z, A, B, C, Robot speed, Motor
veltest = 5;    % Speed mm/s %TEST
velpos = 5;  % mm/s
x1 = 270.00;
x2 = 330.00;
L1 = 20.00;
L2 = 10.00;
L3 = 0.00;
z1 = 88.00;
z2 = 95.00;


M = [
    % First Line (Y = 15, de X=250 a X=340)
    x1,  L1,  z1,  -180.00,  0.00,  270.00,  veltest,    0,  1;   % P1: approach
    x1,  L1,  z1,  -180.00,  0.00,  270.00,  veltest,    1,  0;   % P2: inicio coser
    x2,  L1,  z1,  -180.00,  0.00,  270.00,  veltest,    0,  0;   % P3: fin coser
    
    % Up and move to the next line (Y = 0)
    x2,  L2,  z2,  -180.00,  0.00,  270.00,  veltest,    0,  1;   % P4: sube aguja
    x2,  L2,  z2,  -180.00,  0.00,  90.00,   velpos,     0,  0;   % P5: traslación rápida
    x2,  L2,  z1,  -180.00,  0.00,  90.00,   veltest,    0,  1;   % P6: baja aguja
    
    % Second line (Y = 0, de X=340 a X=250)
    x2,  L2,  z1,  -180.00,  0.00,  90.00,   veltest,    1,  0;   % P7: inicio coser
    x1,  L2,  z1,  -180.00,  0.00,  90.00,   veltest,    0,  0;   % P8: fin coser
    x1,  L2,  z2,  -180.00,  0.00,  90.00,   veltest,     0,  1;   % P4: sube aguja
    x1,  L3,  z2,  -180.00,  0.00,  91.00,   velpos,     0,  0;
    x1,  L3,  z2,  -180.00,  0.00,  270.00,  velpos,     0,  0;   % P9: Traslacion rapida
    x1,  L3,  z1,  -180.00,  0.00,  270.00,  veltest,    0,  1;   % P10: baja aguja

    %Third Line

    x1,  L3,  z1,  -180.00,  0.00,  270.00,   veltest,    1,  0;   % P11: inicio coser
    x2,  L3,  z1,  -180.00,  0.00,  270.00,   veltest,    0,  0;   % P12: fin coser
    
    % Final up
    x2,  L3, z2,  -180.00,  0.00,  270.00,  veltest,    0,  1;   % P13: sube aguja   

];



Points = M;


%%% EXTRACT DATA
nbuff = 1;
PPnum = Points(:,1:6);
vel_prog = Points(:,7);
vel_prog(1) = 5;
extrudeFlag = Points(:,8);
delaytime = Points(:,end);
np = size(PPnum,1);

PPnum = round(PPnum,2);
vel_prog = round(vel_prog,2);
delaytime = round(delaytime,2);

for i = 1:np
    flag = "(6,0)";
    PPstr(i,1) = genP(PPnum(i,:), flag);
end
%%% SEWING PARAMETERS
L_stitch = 10  ; %TEST

STEPS_PER_STITCH_REV = 600;
MAX_SPS = 3000;
MIN_SPS = 200;

RAMP_FRAC = 0.15;
RAMP_DT = 0.05;
RAMP_MAX_T = 0.3;

%% PRE‑COMPUTE SEGMENT DATA (travel time, motor speed, ramps)
% This moves the calculation outside the main loop.
dist_vec = zeros(np-1,1);
travel_time_vec = zeros(np-1,1);
sps_vec = zeros(np-1,1);
t_ramp_vec = zeros(np-1,1);
t_cruise_vec = zeros(np-1,1);

for i = 1:np-1
    p_curr = PPnum(i,1:3);
    p_next = PPnum(i+1,1:3);
    dist = norm(p_next - p_curr);
    vel_robot = vel_prog(i);
    dist_vec(i) = dist;
    if dist > 0 && vel_robot > 0
        travel_time = dist / vel_robot;
        sps = (STEPS_PER_STITCH_REV / L_stitch) * vel_robot;
        sps = max(MIN_SPS, min(MAX_SPS, sps));
        t_ramp = min(travel_time * RAMP_FRAC, RAMP_MAX_T);
        t_cruise = travel_time - 2*t_ramp;
        if t_cruise < 0
            t_cruise = 0;
            t_ramp = travel_time / 2;
        end
        travel_time_vec(i) = travel_time;
        sps_vec(i) = sps;
        t_ramp_vec(i) = t_ramp;
        t_cruise_vec(i) = t_cruise;
    else
        % zero distance or velocity: no sewing
        travel_time_vec(i) = 0;
        sps_vec(i) = 0;
        t_ramp_vec(i) = 0;
        t_cruise_vec(i) = 0;
    end
end

%%% PREPARE ARDUINO
if estrusore && simulo == 0
    fprintf('\n=== MOTOR PREPARATION ===\n');
    writeline(t_arduino, 'D');
    fprintf('Motor DISABLED. Position needle manually.\n');
    input('Press ENTER when ready...', 's');
    pause(0.5);
    writeline(t_arduino, 'E');
    writeline(t_arduino, 'E');
    fprintf('Motor ENABLED.\n');
    pause(0.5);
    writeline(t_arduino, 'S0');
    pause(0.1);
    fprintf('Ready.\n\n');
end

%% TEACH FIRST POINTS
if robot1 && simulo == 0
    fprintf('Teaching robot (np=%d, nbuff=%d)...\n', np, nbuff);
    writeline(t_robot, num2str(np));
    pause(0.5);
    write(t_robot, PPstr(1,1));
    pause(0.5);
    write(t_robot, num2str(vel_prog(1)));
    pause(0.5);
    writeline(t_robot, '0');
    handshake = "READY";
    temp = CONTROL(t_robot, handshake);
    fprintf('Robot: %s\n', temp);
end

%%% MAIN LOOP
data = strings(np,1);
tempovec = zeros(np,2);

% Start process: send '1' to robot (it will move to point 2)
if robot1 && simulo == 0
    writeline(t_robot, '1');
    handshake = "START";
    data(1) = CONTROL(t_robot, handshake);  % arrival at point 2
    tempovec(1,1) = posixtime(datetime('now'));
    fprintf("P-1 reached (point 2)\n");

    handshake = "DONE";          % CHANGE
end

% Loop over points 1 to np-1
for k = 1 : np-1
    % Log arrival time
    if robot1 && simulo == 0
        tempovec(k,1) = posixtime(datetime('now'));
    end

    % Sewing segment
    if estrusore && extrudeFlag(k) == 1 && k < np
        % Use precomputed values for this segment
        dist = dist_vec(k);
        travel_time = travel_time_vec(k);
        sps = sps_vec(k);
        t_ramp = t_ramp_vec(k);
        t_cruise = t_cruise_vec(k);

        fprintf('  Segment P%d→P%d: dist=%.1f mm, vel=%.1f mm/s, t=%.3f s, motor=%.0f pps\n', ...
                k, k+1, dist, vel_prog(k), travel_time, sps);   % uses vel_prog(k) as per precompute

        % Reset Arduino step counter
        writeline(t_arduino, 'Z');

        % Start sewing motor ramp up
        ramparMotor(t_arduino, 0, sps, t_ramp, RAMP_DT);

        % Send the NEXT point and speed to the robot (this is now the immediate target)
        writeline(t_robot, PPstr(k+1,1));
        writeline(t_robot, num2str(vel_prog(k+1)));

        % Maintain constant speed during cruise (if any)
        if t_cruise > 0
            writeline(t_arduino, sprintf('S%.0f', sps));
            pause(t_cruise);
        end

        % Ramp motor down to zero
        ramparMotor(t_arduino, sps, 0, t_ramp, RAMP_DT);

        % Wait for robot to finish the movement (it will send "DONE")
        temp = CONTROL(t_robot, handshake);

        % Finish the current stitch and stop motor automatically
        writeline(t_arduino, 'F');
        fprintf('  Finishing actual stitch...\n');
        fprintf('  Sewing completed.\n');
        tempovec(k,2) = posixtime(datetime('now'));

        % No need for a long extra pause here; the robot is already at the next point
    else
        % No sewing on this segment: move to next point immediately
        if robot1 && simulo == 0
            if k < np
                fprintf("P-%d reached\n", k);
                writeline(t_robot, PPstr(k+1,1));
                writeline(t_robot, num2str(vel_prog(k+1)));
                temp = CONTROL(t_robot, handshake);   % waits for "DONE"
            end
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

%% RETURN TO HOME
if robot1 && simulo == 0
    writeline(t_robot, '1');   % Sends 1
    pause(2);
    fprintf("Robot returning home...\n");
end


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
