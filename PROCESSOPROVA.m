close all; 
clear all;
clc;
%% DECIDO QUALI TOOL AGGIUNTIVI USARE
estrusore=1; 
extrvar=estrusore;%per modificare i loop e tenerli corretti nel teaching
%% GENERAZIONE ADDRESS
%addr="192.168.0.35";
%port=10008;
%t2=ADDRESS(addr,port);
t=[];
%address e port di 8CRL,FRB,ESTRUSORE,PLC
addr=["192.168.0.20","192.168.0.35"];
port=[10007,10008];
if estrusore==1
    addr=[addr,"192.168.0.6"];
    port=[port,1000];
end
for i=1:length(addr)
    t=[t,ADDRESS(addr(i),port(i))];
end
%% ESTRUSORE
clc;
temperatura=23;%gradi centigradi
tempospurgo=5;%secondi
gspurgo=200;%velocità spurgo
data=EXTRSTART(t(3),temperatura,tempospurgo,gspurgo);
%% PUNTI
clc;
Pold=["(550,0,550,90,0,90)(6,0)","(450,0,450,90,0,90)(6,0)"]; %punti home di mitsu grande e piccolo
P=[550,0,550,90,0,90; 450,0,450,90,0,90];
r=50;%mm
np=100;%numero punti
PP=strings(np,length(addr)-extrvar);
xx=0;
for i=1:length(addr)-extrvar
    PP(:,i)=genP(P(i,:),r,np,xx);
end
%% VELOCITA'
vel=["20","20"];%mm/s per i due robot
extr=ones(1,np).*(200+rand(1,np).*100);
extr=round(extr,0);
extrstr=[];
for i=1:np
    extrstr=[extrstr;strcat("G",num2str(extr(i)),"n")];
end
%% TEACHING
str="OK";
for i=1:length(addr)-extrvar
    for k=1:np-1
        data=TEACH(t(i),PP(k,i),vel(i),str);
    end
    data=TEACH(t(i),PP(np,i),vel(i),str);
    fprintf("ROBOT %i : %s \n",i,data);
end
%% START
M=["1","1"];
str="PARTO";
for i=1:length(addr)-extrvar
    data=MOVE(t(i),M(i),str);
    fprintf("ROBOT %i : %s \n",i,data);
end
write(t(3),extrstr(1));
pause(0.01);
%CONTROLLO MOVIMENTO
M=["1","1"];
str="OK";
stp=1;
for k=2:stp:np
    for i=1:length(addr)-extrvar
        data=CONTROL(t(i),str);
    end
    for i=1:length(addr)-extrvar
        write(t(i),M(i));
    end
    if estrusore==1
        write(t(3),extrstr(k));
    end
    fprintf("PUNTI %i fatti \n",k-1);
end
write(t(3),"G0n");
fprintf("PUNTI %i fatti \n",k);