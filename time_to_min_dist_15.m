function time= time_to_min_dist_15(P_ac,V_ac,P_bc,V_bc)

Vx= ((V_ac(1,1)) - (V_bc(1,1)));
Vy= ((V_ac(2,1)) - (V_bc(2,1)));
d=1.5;
a=(Vx^2 + Vy^2);
delta_Px = (P_ac(1,1)-P_bc(1,1));
delta_Py = (P_ac(2,1)-P_bc(2,1));
b=2*(Vx*delta_Px + Vy*delta_Py);
c= delta_Px^2 + delta_Py^2 - d^2;
t1=(-b+(b^2 - (4*a*c))^.5)/(2*a);
t2=(-b-(b^2 - (4*a*c))^.5)/(2*a);

time=min(t1,t2);
if t1<=0 && t2>0
    t=t2;
end
if t2<=0 && t1>0
    t=t1;
end
if t2<=0 && t1<=0
    t=0;
end
end