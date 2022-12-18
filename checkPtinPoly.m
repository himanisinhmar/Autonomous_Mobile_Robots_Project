function inPoly=checkPtinPoly(DP,pGons)

    NOS=size(pGons,1);
    for k=1:NOS
        pgon=pGons(k);
        TFin1 = isinterior(pGons(k),DP(1),DP(2));
        if TFin1==1
            inPoly=1;
            return
        end
    end
    inPoly=0;
end