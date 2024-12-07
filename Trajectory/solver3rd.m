function[Vout,Aout,Xout,Jt,Jout,Lock]= solver3rd(Vout,Aout,Xout,Jt,Lock)

LoopTime = 0.001;
ACC = .5;
DCC = .8;
Vmax = 1;

if Aout <= -DCC
    xcal = (1/24)*DCC^3/Jt^2+Vout^2/(2*DCC) + Xout;
else
    vm = Vout+Aout^2/(Jt*2);
    am = min(DCC,sqrt(abs(vm)*Jt));
    xcal = Aout/Jt*(Vout+(Aout^2)/(3*Jt))+vm*(am/Jt+vm/am)/2 + Xout;
end

if xcal > 0.0 || Lock == 1
    if Aout <= -DCC
        temp = -(1/12)*DCC^3/(Jt^3);
    elseif  DCC*DCC < vm * Jt
        temp = (-6*Vout*(DCC+Aout)^2*Jt - Aout^2*(6*DCC^2+8*DCC*Aout+3*Aout^2))/(12*DCC*Jt^3);
    else
        temp = ((-12*Vout^3)*Jt^3 + (-36*Aout^2*Vout^2)*Jt^2 + (- 27*Aout^4*Vout - 24*Aout*Vout*(am)^(3))*Jt - 16*Aout^3*(am)^(3) - 6*Aout^6)/(24*am^3*Jt^3);
    end
    Jt = Jt - xcal / temp;
    Lock = 1;
    Jout = -1000*(Aout-2/3*(Vout+Aout*0.001)^2/(Xout+4/3*Vout*0.001));
    if Jout < -Jt
        Jout = -Jt*1.0;
    end
    if Jout > Jt*1.1
        Jout = Jt*1.1;
    end
else
    Jout = Jt;
end

if Aout >= 0
    if Jout > 0
        if Vout + Aout * LoopTime > Vmax
            Jout = -Aout * LoopTime;
        else
            dv = Aout*Aout/(2*Jt) + Aout*LoopTime/2;
            if Vout > Vmax - dv
                Jout = -(Aout*Aout)/( 2*(Vmax-Vout)- Aout*0.001);
            else
                if Aout >= ACC
                    Jout = 0;
                    Aout = ACC;
                end
            end
        end
    end
    Jout = mysat(Jout, Jt * 1.1);
else
    if Aout > -DCC || Jout > 0
        Jout = mysat(Jout, Jt * 1.5);
    else
        Jout = 0;
        Aout = -DCC;
    end
end

if Vout < 0 && Lock == 1
    Lock = 2;
end

Xout = Xout + Vout * LoopTime;
Vout = Vout + Aout * LoopTime;
Aout = Aout + Jout * LoopTime;

end

function o = mysat(a,b)
    o = min(max(a, -b), b);
end
