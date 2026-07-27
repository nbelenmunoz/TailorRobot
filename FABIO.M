close all; 
clear; 
clc;    

%% SELECT WHICH TOOLS TO USE
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

%% CONNECTIONS
clc;
if simulo == 0
    tout = 20; conntout = 30;
    if robot1 == 1
        ip_robot = "192.168.0.22";  
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

%% POINTS MATRIX (robot speed = 5 mm/s, stitch length = 10 mm)
% M = X, Y, Z, A, B, C, Robot speed, Motor
veltest = 3;    % Speed mm/s %TEST
velpos = 5;  % mm/s
x1 = 300.00;
x2 = 360.00;
L1 = 20.00;
L2 = 10.00;
L3 = 0.00;
z1 = 90.00;
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

%% EXTRACT DATA
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

%% SEWING PARAMETERS
L_stitch = 10;
STEPS_PER_STITCH_REV = 600;
MAX_SPS = 1800;
MIN_SPS = 180;

MAX_STEP_SPS = 300;   % Maximum allowed step rate per command without killing the motor
DT_STEP = 0.05;       % delta t

dist_vec = zeros(np-1,1);
travel_time_vec = zeros(np-1,1);
sps_vec = zeros(np-1,1);
t_ramp_actual_vec = zeros(np-1,1);  
t_cruise_actual_vec = zeros(np-1,1); 

for i = 1:np-1
    p_curr = PPnum(i,1:3);
    p_next = PPnum(i+1,1:3);
    dist = norm(p_next - p_curr);
    dist_vec(i) = dist;
    vel_robot = vel_prog(i+1);    % CHANGE

    if dist > 0 && vel_robot > 0
        travel_time = dist / vel_robot;
        travel_time_vec(i) = travel_time;
        sps = (STEPS_PER_STITCH_REV / L_stitch) * vel_robot;
        sps = max(MIN_SPS, min(MAX_SPS, sps));

        % Maximum allowed step rate per command
        n_ramp = ceil((sps - MIN_SPS) / MAX_STEP_SPS);
        t_ramp_needed = n_ramp * DT_STEP;

        % If both ramps don't fit in the travel time, reduce the SPS
        if 2 * t_ramp_needed > travel_time
            n_ramp_max = floor(travel_time / (2 * DT_STEP));
            sps = MIN_SPS + n_ramp_max * MAX_STEP_SPS;
            sps = max(MIN_SPS, min(MAX_SPS, sps));
            n_ramp = ceil((sps - MIN_SPS) / MAX_STEP_SPS); %steps
            t_ramp_needed = n_ramp * DT_STEP;
        end
        sps_vec(i) = sps;
        t_ramp_actual_vec(i) = t_ramp_needed;
        t_cruise_actual_vec(i) = travel_time - 2 * t_ramp_needed;
    else
        travel_time_vec(i) = 0;
        sps_vec(i) = 0;
        t_ramp_actual_vec(i) = 0;
        t_cruise_actual_vec(i) = 0;
    end
end

%% PREPARE ARDUINO
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
    fprintf('Teaching robot (np=%d)...\n', np);
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

%% MAIN LOOP
data = strings(np,1);
tempovec = zeros(np,2);

% Start process
if robot1 && simulo == 0
    writeline(t_robot, '1');
    handshake = "START";
    data(1) = CONTROL(t_robot, handshake);
    tempovec(1,1) = posixtime(datetime('now'));
    fprintf("Robot started, entering main loop\n");
    handshake = "DONE";   % CHANGE
end

for k = 1 : np-1
    if robot1 && simulo == 0
        tempovec(k,1) = posixtime(datetime('now'));
    end
    

    if estrusore && extrudeFlag(k) == 1
        fprintf('  Segment P%d->P%d: dist=%.1f mm, vel=%.1f mm/s, t=%.3f s, motor=%.0f pps\n', ...
                k, k+1, dist_vec(k), vel_prog(k+1), travel_time_vec(k), sps_vec(k));
        
        if travel_time_vec(k) > 0
            % Reset Arduino stitch counter
            writeline(t_arduino, 'Z');
            
            % Start robot first
            writeline(t_robot, PPstr(k+1,1));
            writeline(t_robot, num2str(vel_prog(k+1)));
            pause(0.02);
            
            % ramp for accelerating (MIN_SPS to sps)
            ramparMotor(t_arduino, MIN_SPS, sps_vec(k));
            
            % t cruise constant speed
            if t_cruise_actual_vec(k) > 0
                pause(t_cruise_actual_vec(k));
            end
            
            % Ramp for desaccelerating (sps to MIN_SPS)
            ramparMotor(t_arduino, sps_vec(k), MIN_SPS);
            
            % Wait for robot to finish movement
            temp = CONTROL(t_robot, handshake);   % "DONE"
            
            % Finish stitch
            writeline(t_arduino, 'F');
            fprintf('  Sewing completed.\n');
        else
            % zero distance – still advance robot
            writeline(t_robot, PPstr(k+1,1));
            writeline(t_robot, num2str(vel_prog(k+1)));
            temp = CONTROL(t_robot, handshake);
        end
        tempovec(k,2) = posixtime(datetime('now'));
        
    else
        if robot1 && simulo == 0
            fprintf("P-%d reached, moving to P-%d (non-sewing)\n", k, k+1);
            writeline(t_robot, PPstr(k+1,1));
            writeline(t_robot, num2str(vel_prog(k+1)));
            temp = CONTROL(t_robot, handshake);
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
    writeline(t_robot, '1');    % only one '1' is needed now
    pause(2);
    fprintf("Robot returning home...\n");
end

rawdata = data;
save("rawdata.mat", "rawdata");


%% LOCAL FUNCTION
function ramparMotor(t_arduino, v_start, v_end)
    MAX_STEP = 300;  
    DT = 0.05;        

    if v_start == v_end
        return;
    end

    direccion = sign(v_end - v_start);
    n_pasos = ceil(abs(v_end - v_start) / MAX_STEP);

    for i = 1:n_pasos
        v = v_start + direccion * min(i * MAX_STEP, abs(v_end - v_start));
        v = max(0, v);
        writeline(t_arduino, sprintf('S%.0f', v));
        pause(DT);
    end

    writeline(t_arduino, sprintf('S%.0f', v_end));
end
