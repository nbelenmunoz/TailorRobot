function [msg]=PLCCONV(numerical)
    %Genero codifica ad hoc come python
    packedValue = typecast(single(numerical), 'uint8');
    temp=uint8(char("GG"));
    msg = [temp,int8(0), int8(0), packedValue];
end