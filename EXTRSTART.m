function [data]=EXTRSTART(t,temperatura,tempospurgo,gspurgo)
   tempstr=strcat("M",num2str(temperatura),"n");
   write(t,tempstr);
   while true
        data=native2unicode(read(t));
        if length(data)>1
            break
        end
    end
   %data=native2unicode(read(t));
   disp(data);
   pause(0.5);
   write(t,tempstr);
   pause(2.0);
   write(t,"T0n");
   pause(1.0);
   write(t,"T0n");
   temp=native2unicode(read(t));
   while abs(temperatura-temp)>1.0
       pause(2.0);
       %write(t,"T0n");
       temp=native2unicode(read(t));
       disp(temp);
   end
   pause(1.0);
   gspurgostr=strcat("G",num2str(gspurgo),"n");
   write(t,gspurgostr);
   pause(tempospurgo);
   write(t,"G0n");
end