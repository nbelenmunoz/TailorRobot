function [data]=TEACH(t,P,vel,str)
    write(t,P);
    pause(0.5);
    write(t,vel);
    while true
        data=native2unicode(read(t));
        if length(data)>0 %&& convertCharsToStrings(data) == str+char(13)
            break
        end
    end
end
