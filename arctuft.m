close all;
clear;
clc;

%% DECIDO QUALI TOOL USARE
% Metto 0 o 1 se non uso/uso il dispositivo
robots = 1; motors = 0; umotors = 1;
% IP
IP_robot = ["192.168.0.22", 10008];
IP_motor = [];
IP_umotor = ["COM6", 9600];
% Flag robot
robot_flag = "(6,0)";

%% GENERAZIONE ADDRESS
clc;
t_robot = []; t_motor = []; t_umotor = [];
for i = 1:length(robots)
    if robots(i) == 1
        t_robot = [t_robot, ADDRESS(IP_robot(i, 1), IP_robot(i, 2))];
        configureTerminator(t_robot(end), "CR"); % assista ha bisogno di CR specificato
    end
end
for i = 1:length(motors)
    if motors(i) == 1
        t_motor = [t_motor, ADDRESS(IP_motor(i, 1), IP_motor(i, 2))];
    end
end
for i = 1:length(umotors)
    if umotors(i) == 1
        t_umotor = [t_umotor, serialport(IP_umotor(i, 1), str2double(IP_umotor(i, 2)))];
        configureTerminator(t_umotor(end), "LF");
        t_umotor(end).Timeout = 2;
        pause(2);                      
        flush(t_umotor(end));         
    end
end

%% POINTS
n_t = 3;          % Number of rows
radius = 77.5;    % mm
P_start = zeros(n_t, 6); P_end = P_start;
x = 300;
P_start(1,:)=[280,-40,130,-160,0,-90];
P_end(1,:)=[340,-40,130,-120,0,-90];

c = norm(P_end(1, 1:3)-P_start(1,1:3));

A1 = -70;
A2 = -100;
dA = abs(A2-A1);
n_deg = dA;

P_start(1,4) = A1;
P_end (1,4) = A2;
segno=1;
y_offset=10;%mm
for i = 2:n_t
    P_start(i, :)=P_end(i-1, :);
    P_start(i, 2)=P_end(i-1,2)+y_offset;P_start(i, 6)=P_start(i,6)+segno*180; 
    P_start(i,4)=A1;
    P_end(i,:)=P_start(i-1, :);
    P_end(i,2)=P_start(i,2);P_end(i, 6)=P_start(i,6);
    P_end(i,4)=A2;
    segno=-segno;
end



%% CONVERTO NUMERI IN STRINGHE PER TEACH
PPstr = strings(n_t, 2);
for i = 1:n_t
    PPstr(i, 1) = genP(P_start(i, 1:6), robot_flag);   % P1
    PPstr(i, 2) = genP(P_end(i, 1:6),   robot_flag);   % P2
end

clc;
disp(PPstr(1, 1));

%% PARAMETRI CUCITURA
Mvel    = 3;         % TEST
L_stitch    = 10;         % TEST
STEPS_REV   = 600;
MIN_SPS = 10; MAX_SPS = 1800;
MAX_STEP = 300; DT = 0.05;
t_delay = 1;

%% PRE COMPUTING OF LINES
L_arc = radius * deg2rad(n_deg);
T_tot = zeros(n_t, 1); L_tot = T_tot; n_st = T_tot;
sps_vec = T_tot; n_ramp_vec = T_tot; t_cruise = T_tot;
for i = 1:n_t
    L_tot(i) = L_arc;
    T_tot(i) = L_tot(i) / Mvel;
    n_st(i)  = max(1, round(L_tot(i) / L_stitch));      % puntadas enteras
    L_real   = L_tot(i) / n_st(i);
    sps = STEPS_REV / L_real * Mvel;
    sps = max(MIN_SPS, min(MAX_SPS, sps));
    n_ramp = ceil((sps - MIN_SPS) / MAX_STEP);
    if 2 * n_ramp * DT > T_tot(i)                         % las rampas no caben
        n_ramp = max(0, floor(T_tot(i) / (2 * DT)));
        sps = MIN_SPS + n_ramp * MAX_STEP;
    end
    sps_vec(i) = sps; n_ramp_vec(i) = n_ramp;
    t_cruise(i) = max(0, T_tot(i) - 2 * n_ramp * DT);
    fprintf('Arch %d: T=%.2f s, L=%.1f mm, %d stitches, %.0f sps, cruise %.2f s\n', ...
        i, T_tot(i), L_tot(i), n_st(i), sps, t_cruise(i));
end

%% Arduino Preparation
clc;
disp("VAI NEL PROGRAMMA MITSU E AVVIALO");
if umotors == 1
    fprintf('\n=== MOTOR PREPARATION ===\n');
    writeline(t_umotor, 'D');
    fprintf('Motor DISABLED. Position needle manually.\n');
    input('Press ENTER when ready...', 's');
    pause(0.5);
    writeline(t_umotor, 'E');
    writeline(t_umotor, 'E');
    fprintf('Motor ENABLED.\n');
    pause(0.5);
    writeline(t_umotor, 'S0');
    pause(0.1);
    flush(t_umotor);
    fprintf('Ready.\n\n');
end


%% PROCESSO
n_t=3;
flush(t_robot);

% try
for n = 1:n_t  % Send data to the robot
    disp(n);
    pause(0.5);
    writeline(t_robot, PPstr(n, 1));
    data = CONTROL(t_robot); disp(data);   % P1_fatto
    pause(0.5);
    writeline(t_robot, num2str(Mvel));   
    data = CONTROL(t_robot); disp(data);   % Velocity
    pause(0.5);
    writeline(t_robot, PPstr(n, 2));
    data = CONTROL(t_robot); disp(data);   % P2_fatto
    pause(0.5);
    writeline(t_robot, num2str(n_deg));
    data = CONTROL(t_robot); disp(data);   % deg_ok
    writeline(t_robot, num2str(radius));
    data = CONTROL(t_robot); disp(data);   % rad_ok
    pause(0.5);
    writeline(t_robot, "1"); %M2
    % PREPARE THE MOTOR
    if umotors == 1
        flush(t_umotor);
        writeline(t_umotor, 'Z');
        pause(0.1);
        writeline(t_umotor, sprintf('T%d', n_st(n) * STEPS_REV));
        %pause(0.1);
    end
    % ARCH
    pause(0.5);
    writeline(t_robot, "1");%M3
    data = CONTROL(t_robot); disp(data);   % ARCH
    t_sys = tic;

    % SEWING
    if umotors == 1
        pause(t_delay);
        ramparMotor(t_umotor, MIN_SPS, sps_vec(n));
        if t_cruise(n) > 0
            pause(t_cruise(n));
        end
        ramparMotor(t_umotor, sps_vec(n), MIN_SPS);
        t0 = tic;
        ok = false;

        while toc(t0) < 10
            if t_umotor.NumBytesAvailable > 0
                l = readline(t_umotor);
                if contains(l, "completada"), ok = true;
                    break;
                end
            else
                pause(0.02);
            end
        end
        if ~ok
            writeline(t_umotor, 'F'); %pause(4);
        end
    end
    % fprintf('ARCH %d: %.2f s (Theory %.2f s)\n', n, toc(t_sys), T_tot(n));
    % pause(pausa(1));
    data = CONTROL(t_robot); disp(data);   %ARC_DONE
    pause(4);
    writeline(t_robot,'1'); %M4
    data = CONTROL(t_robot); disp(data);   % UP
    pause(0.5);

    %Rotation
    l_r = 1 + mod(n-1, 2);
    disp(l_r);
    if n==n_t %M5
        writeline(t_robot, num2str(0)); %0 the last trace
    else
        writeline(t_robot, num2str(l_r)); %1 or 2 the other cases
    end

    data = CONTROL(t_robot); disp(data);   % FINITO
    pause(0.5);
    if n == n_t %M6
        writeline(t_robot, '1');
    else
        writeline(t_robot, '0');
    end
    data = CONTROL(t_robot); disp(data);   %CONTINUO/HOME
end
% data = CONTROL(t_robot); disp(data);   %HOME
% catch ME
% if umotors == 1
%     writeline(t_umotor, 'S0');pause(0.05);writeline(t_umotor, 'D');
% end
% end

%% FINISH
if umotors == 1
    writeline(t_umotor, 'S0'); pause(0.05);
    writeline(t_umotor, 'D');
end
disp("PROCESS FINISHED");

%% LOCAL FUNCTION
function ramparMotor(t_ard, v_start, v_end)
    MAX_STEP = 300; DT = 0.05;
    if v_start == v_end, return; end
    dir = sign(v_end - v_start);
    n = ceil(abs(v_end - v_start) / MAX_STEP);
    for i = 1:n
        v = max(0, v_start + dir * min(i * MAX_STEP, abs(v_end - v_start)));
        writeline(t_ard, sprintf('S%.0f', v));
        pause(DT);
    end
    writeline(t_ard, sprintf('S%.0f', v_end));
end
