function [data]=PLCSTOP(t)
    temp="ST";temp=uint8(char(temp));
    write(t,temp);
    pause(0.5);
    temp="SS";temp=uint8(char(temp));
    write(t,temp);
    data=0;
end