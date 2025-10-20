function [PPstr]=genP(P,flag)
%ESPORTO
PPstr=[];
gigi=num2str(P(1,1));
for k=2:length(P)
gigi=strcat(gigi,",",num2str(P(1,k)));
end
PPstr=[PPstr;strcat("(",gigi,")",flag)];
end