function [t]=ADDRESS(addr,port,tout,conntout)
%Ping di prova
xxx=strcat("!ping ",addr);
eval(xxx);
pause(0.5);
%CONNETTO
t=tcpclient(addr,port,"Timeout",tout,"ConnectTimeout",conntout);
end