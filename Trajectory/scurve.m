function y = scurve(fref, AMax, JMax, Ts)

p_acc = AMax; p_dcc = -AMax;
p_j1 = JMax; p_j2 = -JMax; p_j3 = -JMax; p_j4 = +JMax;
loop_time = Ts;

persistent fout  aout jout;
if isempty(fout) || isempty(fout) || isempty(fout)
    fout = 0; aout = 0; jout = 0;
end

if (fout < fref)
    jout = p_j1;
    if (aout >= 0)
        dv = abs(aout * aout / (2.0 * p_j2));
        if (fout < (fref - dv))
            if (aout >= p_acc)
                jout = 0; aout = p_acc;
            end
        else
            if (abs(fref - fout) < 0.0001)
                jout = p_j2;
            else
                jout = -abs(0.5 * aout * aout / (fref - fout));
            end
        end
    end
    if (jout < p_j2 * 1.1)
        jout = p_j2 * 1.1;
    end
    if (jout > p_j1 * 1.1)
        jout = p_j1 * 1.1;
    end
end

if (fout > fref)
    jout = p_j3;
    if (aout <= 0)
        dv = abs(aout * aout / (2.0 * p_j4));
        if (fout >= (fref + dv))
            if (aout <= p_dcc)
                jout = 0; aout = p_dcc;
            end
        else
            if (abs(fref - fout) < 0.0001)
                jout = p_j4;
            else
                jout = abs(0.5 * aout * aout / (fref - fout));
            end
        end
    end
    if (jout < p_j3 * 1.1)
        jout = p_j3 * 1.1;
    end
    if (jout > p_j4 * 1.1)
        jout = p_j4 * 1.1;
    end
end

if ((abs(fref - fout) < 0.001) && abs(aout) < 0.001)
    jout = 0;
    aout = 0;
    fout = fref;
else
    aout = aout  + jout * loop_time;
    fout = fout + aout * loop_time;
end

y = fout;

end