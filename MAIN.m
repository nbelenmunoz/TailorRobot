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
robot1=1; estrusore=0; % Removed robot2 and plc
%per modificare i loop e tenerli corretti nel teaching ai soli robot
toolvar=estrusore; % Removed plc
%Per sapere il dispositivo i-esimo che numero è effettuo una cumsum di 1
toolvec=robot1; % Only robot1
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
% X Y Z A B C Vrobot Varduino(200-1000) Tdelay(delay time the robot waits before moving to the next point)
%X Y Z A B C EXTRUDER
M = [
    315.000,  -10.000, 470.000, -180.000, 0.000, 0.000, 10, 0,     1; % P1S
    315.000,  -10.000, 415.000, -180.000, 0.000, 0.000, 20, 0,     2;
    315.000,  -10.000, 415.000, -180.000, 0.000, 0.000, 30, 40,    2;% P1
    315.000,   60.000, 415.000, -180.000, 0.000, 0.000, 40, 40,    2; % P2
    315.000,   60.000, 470.000, -180.000, 0.000, 0.000, 50, 0,     0;
    315.000,   60.000, 470.000, -180.000, 0.000, 0.000, 60, 0,     0;% P2S
];

%Save this matrix as "Points"
Points=M;

%% PUNTI,VEL,tempi
clc;
%Scelgo numero punti buffer
nbuff=1;
%load Points.mat;
PPnum=Points(:,1:6);
vel=Points(:,7:7); % Only robot1 velocity
vel(1,:)=50; %correzione vel avvicinamento a P1
toolvel=Points(:,end-1:end-1); % Only extruder velocity
toolvel(1,:)=0;%correzione tool vel avvicinamento a P1
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

if estrusore==1
    extrstr=[];
    for i=1:length(PPnum(:,1,1))
        extrstr=[extrstr;strcat("G",num2str(toolvel(i,1)),"n")];
    end
end

%% PREPARAZIONE tool
clc;
omega = (pi/4);
extrvel=(10^6)/(4254.77*omega);
if estrusore==1
    temperatura=0;%gradi centigradi
    tempospurgo=2*pi/omega;%secondi, con -1000 uso 6.3 sec
    gspurgo=extrvel;%velocità spurgo
    data=EXTRSTART(t(toolvec(2)),temperatura,tempospurgo,gspurgo); %Activacion motor
end

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
        %data=TEACH(t(1),PPstr(k,1),num2str(vel(k,1)),str);
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
data=strings(length(PPnum(:,1,1)),1); % Only for robot1
tempovec=zeros(length(PPnum(:,1,1)),length(addr));

if estrusore==1 %Parte estrusore
    write(t(toolvec(2)),extrstr(2,:));
    tempovec(k,toolvec(2))= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
end

%CONTROLLO MOVIMENTO
if robot1>0 %Se il robot esiste
    pause(delaytime(1));
    writeline(t(1),num2str(1));
    data(1,1)=CONTROL(t(1),str);
    tempovec(1,1)= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
    fprintf("P-%i \n",1);
end

for k=nbuff+1:length(PPnum(:,1,1))-1
    if robot1>0 %Aggiorno ROBOT
        writeline(t(1),PPstr(k,1));
        writeline(t(1),num2str(vel(k,1)));
    end
    
    if robot1>0 %Ricevo avvenuto arrivo a destinazione del robot
        data(k-nbuff+1,1)=CONTROL(t(1),str);
        tempovec(k-nbuff+1,1)= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
    end
    
    fprintf("P-%i \n",k-nbuff+1);
    
    if estrusore==1 %Aggiorno velocità estrusore
        write(t(toolvec(2)),extrstr(k-nbuff+1,:));
        tempovec(k-nbuff+1,toolvec(2))= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
    end
    
    if robot1>0 %Se il robot esiste
        if k==length(PPnum(:,1,1))
            data(k-nbuff+1,1)=CONTROL(t(1),str);
        end
        pause(delaytime(k-nbuff+1));
        writeline(t(1),num2str(1))
    end
end

for k=2:nbuff
    if estrusore==1 %Aggiorno velocità estrusore
        write(t(toolvec(2)),extrstr(length(PPnum(:,1,1))-nbuff+k,:));
        tempovec(length(PPnum(:,1,1))-nbuff+k,toolvec(2))= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
    end
    
    data(length(PPnum(:,1,1))-nbuff+k,1)=CONTROL(t(1),str);
    fprintf("P-%i \n",length(PPnum(:,1,1))-nbuff+k);
    pause(delaytime(length(PPnum(:,1,1))-nbuff+k));
end

%%FINE DEL PROCESSO
if estrusore==1 %fermo estrusore
    write(t(toolvec(2)),"G0n");
    pause(0.02);
    write(t(toolvec(2)),"G0n");%lo ripeto per sicurezza
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
