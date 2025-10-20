close all; 
clear all;
clc;
%% DECIDO QUALI TOOL USARE
%Simulation or online 
simulo=0;
if simulo==1
    addr=[1,1,1,1]; %non genero address ma vado avanti nei blocchi di programma
end
%Metto 0 o 1 se non uso/uso il dispositivo
robot1=1; robot2=0; estrusore=0; plc=0;
%per modificare i loop e tenerli corretti nel teaching ai soli robot
toolvar=estrusore+plc;
%Per sapere il dispositivo i-esimo che numero è effettuo una cumsum di 1 
toolvec=[robot1,robot1+robot2];toolvec=[toolvec,toolvec(end)+estrusore];toolvec=[toolvec,toolvec(end)+plc];

%% GENERAZIONE ADDRESS (CHECK ROBOT 1 ID AND PORT)
clc;
if simulo==0
    %vettore delle connessioni
    t=[];
    tout=20;conntout=30;%secondi
    %address e port di 8CRL,FRB,ESTRUSORE,PLC
    addr=[]; port=[];
    if robot1==1 %8CRL (GRANDE) e 5ASD (assista)
        addr=[addr,"192.168.0.22"];
        port=[port,10008];
    end
    if robot2==1 %FRB (PICCOLO)
        addr=[addr,"192.168.0.35"];
        port=[port,10008];
    end
    if estrusore==1 %ESTRUSORE
        addr=[addr,"192.168.0.6"];
        port=[port,1000];
    end
    if plc==1 %PLC
        addr=[addr,"192.168.0.42"];
        port=[port,10002];
    end
    %Connetto
    if length(addr)<1 %In caso nesun dispositivo sia dichiarato
        error("NO DEVICE DECLARED");
    else
        for i=1:length(addr) %Generazione del vettore connessioni
            t=[t,ADDRESS(addr(i),port(i),tout,conntout)];
            if i==1
                configureTerminator(t(i),"CR"); %assista ha bisogno di CR specificato
            end
        end
    end
end

%% GENERATE HERE THE POINTS MATRIX
% X Y Z A B C Vrobot Varduino(200-1000) Tdelay(delay time the robot waits before moving to the next point)
M = [
    0.000,   250.000, 300.000,  180.000, 0.000, 0.000, 6, 0, 1;  % P1
    80.000,  210.000, 215.000,  180.000, 0.000, 0.000, 7,   0, 1;  % P2
    80.000,  350.000, 215.000,  180.000, 0.000, 0.000, 7, 0, 1;  % P3
   -80.000,  350.000, 215.000,  180.000, 0.000, 0.000, 7, 0, 1;  % P4
   -80.000,  210.000, 215.000, -180.000, 0.000, 0.000, 7, 0, 1;  % P5
];


%Save this matrix as "Points"
Points=M;

%% PUNTI,VEL,tempi
clc;
%Scelgo numero punti buffer
nbuff=2;
%load Points.mat;
PPnum=Points(:,1:6);
vel=Points(:,7:7+toolvec(end)-toolvar); vel(1,:)=50; %correzione vel avvicinamento a P1
toolvel=Points(:,end-1-toolvar:end-1); toolvel(1,:)=0;%correzione tool vel avvicinamento a P1
delaytime=Points(:,end); 
np=length(PPnum(:,1));%numero finale di punti
%Approssimo
approx=2;
PPnum=round(PPnum,2);vel=round(vel,approx);delaytime=round(delaytime,2);
%CONVERTO A STRINGHE CORRETTE PER IL TEACHING
for i=1:length(PPnum(:,1))
    for k=1:toolvec(end)-toolvar
        if k==1
            flag="(7,0)"; %flag robot 1
        elseif k==2
            flag="(6,0)"; %flag robot 2
        end
        PPstr(i,k)=genP(PPnum(i,:,k),flag); %stringa punti robot
    end
end
if estrusore==1 
    extrstr=[];
    for i=1:length(PPnum(:,1,1))
        extrstr=[extrstr;strcat("G",num2str(toolvel(i,1)),"n")];
    end
end
if plc==1 
    plcstr=[];
    for i=1:length(PPnum(:,1,1))
        plcstr=[plcstr;PLCCONV(toolvel(i,2))];
    end
end

%% PREPARAZIONE tool
clc;
omega = (pi/4);
extrvel=(10^6)/(4*254.77*omega);
if estrusore==1 
    temperatura=0;%gradi centigradi
    tempospurgo=2*pi/omega;%secondi, con -1000 uso 6.3 sec
    gspurgo=extrvel;%velocità spurgo
    data=EXTRSTART(t(toolvec(3)),temperatura,tempospurgo,gspurgo);
end
%%
clc;
if plc==1
    data=PLCSTART(t(toolvec(4)));
end
%Suggerimenti
if toolvec(end)-toolvar>0 %Se i robot esistono
    fprintf("Be sure that PP and MVEL on Mitsubishi are at least long as nbuff(%i) \n",nbuff);
    disp("Servo On, Start MOV1 and MOV2 on the robots.");
    disp("Wait the movement finish of the first robot to move the second etc.");
end

%% TEACHING DEI PRIMI PUNTI
if toolvec(end)-toolvar>0 %Se i robot esistono
    %teach np to the robots
    for i=1:toolvec(end)-toolvar
        writeline(t(i),num2str(np)); %scrivo np
    end
    pause(0.2);
    for i=1:toolvec(end)-toolvar
        writeline(t(i),num2str(nbuff)); %scrivo nbuffer
    end
    %teach P1-Pk to the robots
    for i=1:toolvec(end)-toolvar
        for k=1:nbuff
            str="P-fatto";
            %data=TEACH(t(i),PPstr(k,i),num2str(vel(k,i)),str);
            write(t,PPstr(k,i));
            pause(0.5);
            write(t,num2str(vel(k,i)))
            pause(1);
        end
    end
    pause(0.5);
    %robots move to P1
    for i=1:toolvec(end)-toolvar
        writeline(t(i),num2str(0));
    end
    for i=1:toolvec(end)-toolvar
        temp=CONTROL(t(i),str);
        fprintf("ROBOT %i: %s \n",i,temp);
    end
end

%% PROCESSO
%https://it.mathworks.com/help/parallel-computing/quick-start-parallel-computing-in-matlab.html
% parfor i = 1:3
%     c(i) = max(eig(rand(1000)));
% end
k=1;
data=strings(length(PPnum(:,1,1)),toolvec(end)-toolvar);
tempovec=zeros(length(PPnum(:,1,1)),length(addr));
if estrusore==1 %Parte estrusore
    write(t(toolvec(3)),extrstr(2,:));
    tempovec(k,toolvec(3))= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
end
if plc==1 %Parte PLC
    write(t(toolvec(4)),plcstr(2,:));
    tempovec(k,toolvec(4))= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
end
%CONTROLLO MOVIMENTO
if toolvec(end)-toolvar>0 %Se i robot esistono
    pause(delaytime(1));
    for i=1:toolvec(end)-toolvar
        writeline(t(i),num2str(1));
        data(1,i)=CONTROL(t(i),str);
        tempovec(1,i)= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
        fprintf("P-%i \n",1);
    end
end
for k=nbuff+1:length(PPnum(:,1,1))-1
    for i=1:toolvec(end)-toolvar %Aggiorno ROBOT, che continuano
        writeline(t(i),PPstr(k,i));
        writeline(t(i),num2str(vel(k,i)));
    end
    for i=1:toolvec(end)-toolvar %Ricevo avvenuto arrivo a destinazione dei robot
        data(k-nbuff+1,i)=CONTROL(t(i),str);
        tempovec(k-nbuff+1,i)= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
    end
    fprintf("P-%i \n",k-nbuff+1);
    if estrusore==1 %Aggiorno velocità estrusore
        write(t(toolvec(3)),extrstr(k-nbuff+1,:));
        tempovec(k-nbuff+1,toolvec(3))= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
    end
    if plc==1 %Aggiorno velocità PLC
        write(t(toolvec(4)),plcstr(k-nbuff+1,:));
        tempovec(k-nbuff+1,toolvec(4))= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
    end
    if toolvec(end)-toolvar==0 %Se i robot non esistono
        pause(1);
    elseif toolvec(end)-toolvar >0 %Se i robot esistono
        if k==length(PPnum(:,1,1))
            data(k-nbuff+1,i)=CONTROL(t(i),str);
        end
        pause(delaytime(k-nbuff+1));
    end
end
for k=2:nbuff
    if estrusore==1 %Aggiorno velocità estrusore
        write(t(toolvec(3)),extrstr(length(PPnum(:,1,1))-nbuff+k,:));
        tempovec(length(PPnum(:,1,1))-nbuff+k,toolvec(3))= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
    end
    if plc==1 %Aggiorno velocità PLC
        write(t(toolvec(4)),plcstr(length(PPnum(:,1,1))-nbuff+k,:));
        tempovec(length(PPnum(:,1,1))-nbuff+k,toolvec(4))= seconds(duration(string(datetime('now','Format','HH:mm:ss.SS'))));
    end
    data(length(PPnum(:,1,1))-nbuff+k,i)=CONTROL(t(i),str);
    fprintf("P-%i \n",length(PPnum(:,1,1))-nbuff+k);
    pause(delaytime(length(PPnum(:,1,1))-nbuff+k));
end
%%FINE DEL PROCESSO
if plc==1 %fermo PLC
    temp=PLCSTOP(t(toolvec(4)));
end
if estrusore==1 %fermo estrusore
    write(t(toolvec(3)),"G0n");
    pause(0.02);
    write(t(toolvec(3)),"G0n");%lo ripeto per sicurezza
end
fprintf("PROCESSO TERMINATO \n",k);

%% I robot si fermano a fine traiettoria dal Mitsubishi, rimangono in attesa di ok dal matlab
tornoacasa=1;%se vuoi che il robot torni in home prima di spegnersi
M=repmat(num2str(tornoacasa),1,length(addr)-toolvar);%ripeto la stinga di "1" per comunicare con i robot
if toolvec(end)-toolvar >0 %Se i robot esistono
    str="PARTO";
    for i=1:toolvec(end)-toolvar %Faccio tornare a Phome i robot
        temp=MOVE(t(i),M(i),str);
        fprintf("ROBOT %i : torna a casa \n",i);
    end
end

%% Salvo i dati in raw format 
rawdata=data;
save("rawdata.mat","rawdata");


