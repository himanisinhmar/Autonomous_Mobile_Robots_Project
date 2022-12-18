function dp=findSpotOnLine(out1)
%     
%     [m i]= max(isnan(in));
%     i(1)
    if isempty(out1)
        disp('Could not find a valid point on the line')
        dp=[nan nan];
        return
    else
        dp=[(out1(1,1)+out1(2,1))/2 (out1(1,2)+out1(2,2))/2]; 
    end
end