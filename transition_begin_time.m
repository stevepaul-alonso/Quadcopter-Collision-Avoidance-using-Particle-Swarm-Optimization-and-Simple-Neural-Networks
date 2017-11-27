function t= transition_begin_time(V1,V2,Pa_i,Pb_i)
% global V1 V2 Pa_i Pb_i
d=4;
% Vx= (abs(V1(1,1)) + abs(V2(1,1)));
% Vy= abs(V1(2,1)) + abs(V2(2,1));

Vx= ((V1(1,1)) - (V2(1,1)));
Vy= ((V1(2,1)) - (V2(2,1)));

a=(Vx^2 + Vy^2);
delta_Px = (Pa_i(1,1)-Pb_i(1,1));
delta_Py = (Pa_i(2,1)-Pb_i(2,1));
b=2*(Vx*delta_Px + Vy*delta_Py);
c= delta_Px^2 + delta_Py^2 - d^2;
t1=(-b+(b^2 - (4*a*c))^.5)/(2*a);
t2=(-b-(b^2 - (4*a*c))^.5)/(2*a);

t=min((t1),(t2));%change here
if t1<=0 && t2>0
    t=t2;
end
if t2<=0 && t1>0
    t=t1;
end
if t2<=0 && t1<=0
    t=0;
end
% if t<1
%     t=0;
% end

end