function [detect tc]=col_det2(P1,V1,P2,V2)%,P3,V3)
% px=P2(1,1)-P1(1,1);
% py=P2(2,1)-P1(2,1);
% vx=V2(1,1)-V1(1,1);
% vy=V2(2,1)-V1(2,1);
% detect=0;
% tc= max(-(px*vx + py*vy)/(vx^2 + vy^2),0);
% P1n=P1 + tc.*V1;
% P2n=P2 + tc.*V2;
detect=0;
[tc d]= minimum_sep(P1,V1,P2,V2);
if  d<1.5      %norm(P1n-P2n)<1.5
    detect=1;
%     tc=(4/(((abs(V1(1,1))+abs(V2(1,1)))^2)+((abs(V1(2,1))+abs(V2(2,1)))^2)))^.5;

end

% px=P3(1,1)-P1(1,1);
% py=P3(2,1)-P1(2,1);
% vx=V3(1,1)-V1(1,1);
% vy=V3(2,1)-V1(2,1);
% detect=0;
% tc2= max(-(px*vx + py*vy)/(vx^2 + vy^2),0);
% P1n=P1 + tc2.*V1;
% P3n=P2 + tc2.*V3;
% if norm(P1n-P3n)<1.5
%     detect=1;
% end
% 
% tc=min(tc,tc2);
end
