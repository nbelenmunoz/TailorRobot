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

%% POINTS MATRIX (robot speed = 5 mm/s, stitch length = 10 mm)
% M = X, Y, Z, A, B, C, Robot speed, Motor
veltest = 10;    % Speed mm/s
velpos = 2;  % mm/s
x1 = 310.00;
x2 = 370.00;
L1 = 25.00;
L2 = 15.00;
L3 = 5.00;
z1 = 90.00;
z2 = 95.00;


M = [
    % First Line (Y = 15, de X=250 a X=340)
    x1,  L1,  z1,  -180.00,  0.00,  270.00,  veltest,    0,  1;   % P1: approach
    x1,  L1,  z1,  -180.00,  0.00,  270.00,  veltest,    1,  0;   % P2: inicio coser
    x2,  L1,  z1,  -180.00,  0.00,  270.00,  veltest,    0,  0;   % P3: fin coser
    
    % Up and move to the next line (Y = 0)
    x2,  L2,  z2,  -180.00,  0.00,  270.00,  veltest,    0,  1;   % P4: sube aguja
    x2,  L2,  z2,  -180.00,  0.00,  90.00,   velpos,  0,  0;   % P5: traslación rápida
    x2,  L2,  z1,  -180.00,  0.00,  90.00,   veltest,    0,  1;   % P6: baja aguja
    
    % Second line (Y = 0, de X=340 a X=250)
    x2,  L2,  z1,  -180.00,  0.00,  90.00,   veltest,    1,  0;   % P7: inicio coser
    x1,  L2,  z1,  -180.00,  0.00,  90.00,   veltest,    0,  0;   % P8: fin coser
    x1,  L2,  z2,  -180.00,  0.00,  90.00,  veltest,     0,  1;   % P4: sube aguja
    x1,  L3,  z2,  -180.00,  0.00,  91.00,   velpos,  0,  0;
    x1,  L3,  z2,  -180.00,  0.00,  270.00,  velpos,  0,  0;   % P9: Traslacion rapida
    x1,  L3,  z1,  -180.00,  0.00,  270.00,   veltest,    0,  1;   % P10: baja aguja

    %Third Line

    x1,  L3,  z1,  -180.00,  0.00,  270.00,   veltest,    1,  0;   % P11: inicio coser
    x2,  L3,  z1,  -180.00,  0.00,  270.00,   veltest,    0,  0;   % P12: fin coser
    
    % Final up
    x2,  L3, z2,  -180.00,  0.00,  270.00,  veltest,    0,  1;   % P13: sube aguja
   

];



Points = M;


%% EXTRACT DATA
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
%% SEWING PARAMETERS
L_stitch = 5   ; %TEST

STEPS_PER_STITCH_REV = 600;
MAX_SPS = 3000;
MIN_SPS = 200;
RAMP_FRAC = 0.15;
RAMP_DT = 0.05;
RAMP_MAX_T = 0.30;




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
    fprintf('Teaching robot (np=%d, nbuff=%d)...\n', np, nbuff);
    writeline(t_robot, num2str(np+5));
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
    writeline(t_robot, '0');
    handshake = "P-fatto";
    temp = CONTROL(t_robot, handshake);
    fprintf('Robot: %s\n', temp);
end

%% MAIN LOOP
data = strings(np,1);
tempovec = zeros(np,2);

% Start process: send '1' to robot (it will move to point 2)
if robot1 && simulo == 0
    writeline(t_robot, '1');
    data(1) = CONTROL(t_robot, handshake);  % arrival at point 2
    tempovec(1,1) = posixtime(datetime('now'));
    fprintf("P-1 reached (point 2)\n");
end

% Loop over points 2 to np-1
for k = 2 : np-1
    % Wait for arrival at current point k (already handled for k=2 above)
    if k > 2
        if robot1 && simulo == 0
            data(k) = CONTROL(t_robot, handshake);
            tempovec(k,1) = posixtime(datetime('now'));
        end
        fprintf("P-%d reached\n", k);
        %input("Enter");
    end
    
    %  stitch start
        %  stitch start
    if estrusore && extrudeFlag(k) == 1 && k < np
        % Calculate distance to next point
        p_curr = PPnum(k,1:3);
        p_next = PPnum(k+1,1:3);
        dist = norm(p_next - p_curr);
        vel_robot = vel_prog(k);
        
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
            
            fprintf('  Segment P%d→P%d: dist=%.1f mm, vel=%.1f mm/s, t=%.3f s, motor=%.0f pps\n', ...
                    k, k+1, dist, vel_robot, travel_time, sps);
            
            % Send next point and speed (for point k+1) to robot buffer
            if robot1 && simulo == 0
                writeline(t_robot, PPstr(k+1,1));
                writeline(t_robot, num2str(vel_prog(k+1)));
            end
            
            % MODIFICADO: Reiniciar contador de pasos en Arduino antes de empezar a coser
            writeline(t_arduino, 'Z');
            pause(0.01);
            
            % Send CONTINUE to robot (start moving to next point)
            writeline(t_robot, '1');
            pause(0.02);
            
            % Start sewing motor with ramps
            ramparMotor(t_arduino, 0, sps, t_ramp, RAMP_DT);
            if t_cruise > 0
                writeline(t_arduino, sprintf('S%.0f', sps));
                pause(t_cruise);
            end
            
            ramparMotor(t_arduino, sps, MIN_SPS, t_ramp, RAMP_DT);
            % Enviar comando F para que complete la puntada y pare automáticamente
            writeline(t_arduino, 'F');
            fprintf('  Finalizando puntada actual...\n');
            % (Opcional) Leer confirmación del Arduino
            % resp = readline(t_arduino);
            % disp(resp);
            
            fprintf('  Sewing completed.\n');
            tempovec(k,2) = posixtime(datetime('now'));
        else
            fprintf('  Zero distance/velocity, skipping sewing.\n');
            writeline(t_robot, '1');
        end
    else
        % No sewing on this segment: just send CONT to continue
        if robot1 && simulo == 0
            % Send next point and speed for k+1 if needed
            if k < np
                writeline(t_robot, PPstr(k+1,1));
                writeline(t_robot, num2str(vel_prog(k+1)));
            end
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

%% RETURN TO HOME (send '1' to robot)
if robot1 && simulo == 0
    writeline(t_robot, '1');
    writeline(t_robot, '1');
    pause(2);  % give time to mov   e home
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
