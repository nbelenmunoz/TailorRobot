close all;
clear all;
clc;

%% DECIDO QUALI TOOL USARE
%Simulation or online
simulo=0;
if simulo==1
    addr=[1,1]; % Only robot1 and extruder addresses
end
%Metto 0 o 1 se non uso/uso il dispositivo
robot1=1; estrusore=0; % Enable extruder
%per modificare i loop e tenerli corretti nel teaching ai soli robot
toolvar=estrusore;
%Per sapere il dispositivo i-esimo che numero è effettuo una cumsum di 1
toolvec=robot1;
if estrusore==1
    toolvec=[toolvec,toolvec(end)+estrusore]; % Add extruder if enabled
end

%% GENERAZIONE ADDRESS (CHECK ROBOT 1 ID AND PORT)
clc;
if simulo==0
    %vettore delle connessioni
    t=[];
    tout=20;conntout=30;%secondi
    %address e port di ROBOT1 e ESTRUSORE
    addr=[]; port=[];
    if robot1==1 %ROBOT1
        addr=[addr,"192.168.0.22"];
        port=[port,10008];
    end
    if estrusore==1 %ESTRUSORE
        addr=[addr,"192.168.0.6"];
        port=[port,1000];
    end
    
    %Connetto
    if length(addr)<1 %In caso nesun dispositivo sia dichiarato
        error("NO DEVICE DECLARED");
    else
        for i=1:length(addr) %Generazione del vettore connessioni
            t=[t,ADDRESS(addr(i),port(i),tout,conntout)];
            if i==1
                configureTerminator(t(i),"CR"); %robot1 ha bisogno di CR specificato
            end
        end
    end
end

    %% GENERATE HERE THE POINTS MATRIX
% X Y Z A B C Vrobot ExtruderActivation(0=no, 1=yes) Tdelay(delay time before moving to next point)

M = [
    300.000,  -20.000, 490.000, -180.000, 0.000, 0.000, 100, 0, 1;
    300.000,  -20.000, 490.000, -180.000, 0.000, 0.000, 100, 1, 1;
    300.000,    0.000, 490.000, -180.000, 0.000, 0.000, 100, 1, 1; 
    300.000,   20.000, 490.000, -180.000, 0.000, 0.000, 100, 1, 1;
    300.000,   40.000, 490.000, -180.000, 0.000, 0.000, 100, 1, 1;
    300.000,   60.000, 490.000, -180.000, 0.000, 0.000, 100, 1, 1; 
    300.000,   60.000, 490.000, -180.000, 0.000, -90.000, 100, 0, 1;
    310.000,   60.000, 490.000, -180.000, 0.000, -90.000, 100, 0, 1;
    310.000,   60.000, 490.000, -180.000, 0.000, -90.000, 100, 1, 1;
    310.000,   60.000, 490.000, -180.000, 0.000, -180.000, 100, 0, 1;
    310.000,   40.000, 490.000, -180.000, 0.000, -180.000, 100, 1, 1;
    310.000,   20.000, 490.000, -180.000, 0.000, -180.000, 100, 1, 1;
    310.000,   0.000, 490.000,  -180.000, 0.000, -180.000, 100, 1, 1;
    310.000,  -20.000, 490.000, -180.000, 0.000, -180.000, 100, 1, 1;

];

%Save this matrix as "Points"
Points=M;

%% PUNTI,VEL,tempi
clc;
%Scelgo numero punti buffer
nbuff=1;
%load Points.mat;
PPnum=Points(:,1:6);
vel=Points(:,7); % Only robot1 velocity
vel(1,:)=50; %correzione vel avvicinamento a P1
extrudeFlag=Points(:,8); % Extruder activation flag (0=no, 1=yes)
delaytime=Points(:,end);
np=length(PPnum(:,1));%numero finale di punti

%Approssimo
approx=2;
PPnum=round(PPnum,2);vel=round(vel,approx);delaytime=round(delaytime,2);

%CONVERTO A STRINGHE CORRETTE PER IL TEACHING
for i=1:length(PPnum(:,1))
    for k=1:1 % Only robot1
        flag="(6,0)"; %flag robot 1
        PPstr(i,k)=genP(PPnum(i,:,k),flag); %stringa punti robot
    end
end

%% EXTRUDER PARAMETERS - FIXED MOVEMENT
% These parameters define the fixed 4-revolution movement
G_value = 50;          % Command: "G200n"
Nrev_axis = 4;          % Number of revolutions
k_axis = 0.0096;        % Constant for time calculation
Trev_axis = G_value * k_axis;         % Time for one revolution
total_extrude_time = Nrev_axis * Trev_axis;   % Total time for 4 revolutions

fprintf("Extruder fixed movement: %.2f rev at G%dn -> %.3f s\n", ...
        Nrev_axis, G_value, total_extrude_time);

%% PREPARAZIONE tool
%Suggerimenti
if robot1>0 %Se il robot esiste
    fprintf("Be sure that PP and MVEL on Mitsubishi are at least long as nbuff(%i) \n",nbuff);
    disp("Servo On, Start MOV1 and MOV2 on the robot.");
end

%% TEACHING DEI PRIMI PUNTI
if robot1>0 %Se il robot esiste
    %teach np to the robot
    writeline(t(1),num2str(np)); %scrivo np
    pause(0.2);
    writeline(t(1),num2str(nbuff)); %scrivo nbuffer
    
    %teach P1-Pk to the robot
    pause(1);
    for k=1:nbuff
        str="P-fatto";
        write(t(1),PPstr(k,1));
        pause(0.5);
        write(t(1),num2str(vel(k,1)))
        pause(1);
    end
    
    pause(0.5);
    %robot moves to P1
    writeline(t(1),num2str(0));
    temp=CONTROL(t(1),str);
    fprintf("ROBOT 1: %s \n",temp);
end

%% PROCESSO
k=1;
data = strings(length(PPnum(:,1,1)),1);
tempovec = zeros(length(PPnum(:,1,1)),length(addr));

% CONTROLLO MOVIMENTO
if robot1>0 %Se il robot esiste
    pause(delaytime(1));

    startCmd = num2str(1);
    fprintf('TX robot1 (START): %s\n', startCmd);
    writeline(t(1), startCmd);

    data(1,1) = CONTROL(t(1),str);
    tempovec(1,1) = seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
    fprintf("P-%i \n",1);
end

for k = nbuff+1 : length(PPnum(:,1,1)) - 1
    if robot1>0 %Aggiorno ROBOT
        % Posizione
        posCmd = PPstr(k,1);
        fprintf('k=%d | TX robot1 (PP): %s\n', k, char(posCmd));
        writeline(t(1), posCmd);

        % Velocità
        velCmd = num2str(vel(k,1));
        fprintf('k=%d | TX robot1 (VEL): %s\n', k, velCmd);
        writeline(t(1), velCmd);
    end
    
    if robot1>0 %Ricevo avvenuto arrivo a destinazione del robot
        data(k-nbuff+1,1) = CONTROL(t(1),str);
        tempovec(k-nbuff+1,1) = seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
    end
    
    fprintf("P-%i \n", k-nbuff+1);
    
    % EXTRUDER CONTROL - FIXED 4-REVOLUTION MOVEMENT
    if estrusore==1 && extrudeFlag(k) == 1
        fprintf('Activating extruder at point P-%i for fixed 4 revolutions (%.3f seconds)\n', ...
                k-nbuff+1, total_extrude_time);
        
        % Start extruder with fixed G200 command
        extrStartCmd = sprintf("G%dn", G_value);
        fprintf('TX estrusore START [%d]: %s\n', toolvec(2), char(extrStartCmd));
        write(t(toolvec(2)), extrStartCmd);
        tempovec(k-nbuff+1,toolvec(2)) = seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
        
        % Wait for the fixed 4-revolution time
        fprintf('Waiting %.3f seconds for %d revolutions...\n', total_extrude_time, Nrev_axis);
        pause(total_extrude_time);
        
        % Stop extruder
        extrStopCmd = "G0n";
        fprintf('TX estrusore STOP [%d]: %s\n', toolvec(2), char(extrStopCmd));
        write(t(toolvec(2)), extrStopCmd);
        
        fprintf('Extrusion completed at point P-%i\n', k-nbuff+1);
    end
    
    if robot1>0 %Se il robot esiste
        if k == length(PPnum(:,1,1))
            data(k-nbuff+1,1) = CONTROL(t(1),str);
        end
        pause(delaytime(k-nbuff+1));

        contCmd = num2str(1);
        fprintf('k=%d | TX robot1 (CONT): %s\n', k, contCmd);
        writeline(t(1), contCmd);
    end
end

%% FINE DEL PROCESSO
if estrusore==1 %fermo estrusore
    stopCmd = "G0n";
    fprintf('TX estrusore stop [%d]: %s\n', toolvec(2), char(stopCmd));
    write(t(toolvec(2)), stopCmd);
    pause(0.02);
    fprintf('TX estrusore stop (repeat) [%d]: %s\n', toolvec(2), char(stopCmd));
    write(t(toolvec(2)), stopCmd); % lo ripeto per sicurezza
end
fprintf("PROCESSO TERMINATO \n");

%% Il robot si ferma a fine traiettoria dal Mitsubishi, rimane in attesa di ok dal matlab
tornoacasa=1;%se vuoi che il robot torni in home prima di spegnersi
if robot1>0 %Se il robot esiste
    str="PARTO";
    temp=MOVE(t(1),num2str(tornoacasa),str);
    fprintf("ROBOT 1 : torna a casa \n");
end

%% Salvo i dati in raw format
rawdata=data;
save("rawdata.mat","rawdata");
