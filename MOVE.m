function [data]=MOVE(t,M,str)
    write(t,M);
    while true
        data=native2unicode(read(t));
        if length(data)>1 && convertCharsToStrings(data) == str + char(13)
            break
        end
    end
end