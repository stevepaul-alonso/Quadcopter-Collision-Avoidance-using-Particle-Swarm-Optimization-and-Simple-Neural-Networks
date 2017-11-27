function minsep = simu(Pa,Va,Pb,Vb)
tmax=20;
dt=.1;
t=0;
sl=0;
while t<=tmax
    t=t+dt;
    sl=sl+1;
    Pa=Pa + Va.*dt;
    Pb= Pb + Vb.*dt;
    sep(sl,1)=norm(Pa-Pb);
end
minsep=min(sep);
end
