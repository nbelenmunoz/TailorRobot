function [data]=PLCSTART(t)
    temp="0";
    write(t,temp);
    while true
        data=native2unicode(read(t));
        if length(data)>1
            disp(data);
            break
        end
    end
    pause(1.0);
    temp="SS";temp=uint8(char(temp));
    write(t,temp);
    pause(1.0);
    temp="HH";temp=uint8(char(temp));
    write(t,temp);
    data=0;
end