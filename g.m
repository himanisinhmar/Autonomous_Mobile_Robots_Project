function Xupdated = g(Xprev,dControl,angleControl)

theta = Xprev(3);

if (angleControl~=0)
    Xupdated = Xprev + [-(dControl/angleControl)*sin(theta) + (dControl/angleControl)*sin(theta+angleControl);
                  (dControl/angleControl)*cos(theta) - (dControl/angleControl)*cos(theta+angleControl);
                  angleControl];
else
    Xupdated = Xprev + [dControl*cos(theta);
                  dControl*sin(theta)'
                  0];
end

end

