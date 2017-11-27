function [tc min_sep]= minimum_sep(P1,Va,P2,Vb)
px=P2(1,1)-P1(1,1);
py=P2(2,1)-P1(2,1);
vx=Vb(1,1)-Va(1,1);
vy=Vb(2,1)-Va(2,1);
%detect=0;
tc= max(-(px*vx + py*vy)/(vx^2 + vy^2),0);
P1n=P1 + tc.*Va;
P2n=P2 + tc.*Vb;
min_sep=norm(P1n-P2n);
y=[tc min_sep];
end