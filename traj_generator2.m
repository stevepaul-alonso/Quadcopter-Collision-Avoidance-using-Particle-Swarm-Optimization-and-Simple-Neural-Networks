function [ desired_state ] = traj_generator(t, state, waypoints,time_int,tm,V1)

persistent waypoints0 traj_time  alpha alpha2 alpha3 n S tmax %d0
if nargin==6 %length(state)==0 
   tmax=tm;
   waypoints0 = waypoints;
   [n1 n2]=size(waypoints0);
   n=n2-1;
   traj_time=time_int';%[0:tmax/(n):tmax];
   traj_time(1,end)=traj_time(1,end);
   S=zeros(n2,1);
   S=traj_time';
   avgvel=(waypoints0(:,end)-waypoints0(:,1))./tmax;
%%
   A=zeros(8*n,8*n);
   b=zeros(8*n,1);
   for i=1:n
       A(i,(i-1)*8+1)=1;
       b(i,1)=waypoints0(1,i);
   end
% n constraints done 
   for i=n:2*n-1
       t1=rem(i,n);
       for j=t1*8+1:t1*8+8
           A(i+1,j)=1;
       end
       b(i+1,1)=waypoints0(1,t1+2);
   end
% 2n constraints done
A(2*n+1,2)=1/(S(2,1)-S(1,1));
b(2*n+1,1)=V1(1,1);%0;
for i=2:8
A(2*n+2,(n-1)*8+i)=(i-1)/(S(n+1,1)-S(n,1));
end
b(2*n+2,1)=V1(1,1);
% 2n+2 constraints are done
cn=0; %for velocity
for i=2*n+3:3*n+1
    cn=cn+1;
    pw=2;
    for j=(cn-1)*8+2:(cn-1)*8+8
        A(i,j)=(pw-1)/(S(cn+1,1)-S(cn,1));
        pw=pw+1;
    end
     j=cn*8 +2;
     A(i,j)=-1/(S(cn+2,1)-S(cn+1,1));
     
     b(i,1)=0;
end
%3n+1 constraints done

%for acceleration
cn=0; 
for i=3*n+2:4*n
    cn=cn+1;
    pw=3;
    for j=(cn-1)*8+3:(cn-1)*8+8
        A(i,j)=(pw-1)*(pw-2)/((S(cn+1,1)-S(cn,1))^2);
        pw=pw+1;
    end
     j=cn*8 +3;
     A(i,j)=-(2/(S(cn+2,1)-S(cn+1,1))^2);
     
     b(i,1)=0;
end
% 4n constraints done
cn=0; 
for i=4*n+1:5*n-1
    cn=cn+1;
    pw=4;
    for j=(cn-1)*8+4:(cn-1)*8+8
        A(i,j)=(pw-1)*(pw-2)*(pw-3)/((S(cn+1,1)-S(cn,1))^3);
        pw=pw+1;
    end
     j=cn*8 +4;
     A(i,j)=-(6/(S(cn+2,1)-S(cn+1,1))^3);
     
     b(i,1)=0;
end


cn=0; 
for i=5*n:6*n-2
    cn=cn+1;
    pw=5;
    for j=(cn-1)*8+5:(cn-1)*8+8
        A(i,j)=(pw-1)*(pw-2)*(pw-3)*(pw-4)/((S(cn+1,1)-S(cn,1))^4);
        pw=pw+1;
    end
     j=cn*8 +5;
     A(i,j)=-(24/(S(cn+2,1)-S(cn+1,1))^4);
     
     b(i,1)=0;
end

cn=0; 
for i=6*n-1:7*n-3
    cn=cn+1;
    pw=6;
    for j=(cn-1)*8+6:(cn-1)*8+8
        A(i,j)=(pw-1)*(pw-2)*(pw-3)*(pw-4)*(pw-5)/((S(cn+1,1)-S(cn,1))^5);
        pw=pw+1;
    end
     j=cn*8 +6;
     A(i,j)=-(120/(S(cn+2,1)-S(cn+1,1))^5);
     
     b(i,1)=0;
end

cn=0; 
for i=7*n-2:8*n-4
    cn=cn+1;
    pw=7;
    for j=(cn-1)*8+7:(cn-1)*8+8
        A(i,j)=(pw-1)*(pw-2)*(pw-3)*(pw-4)*(pw-5)*(pw-6)/((S(cn+1,1)-S(cn,1))^6);
        pw=pw+1;
    end
     j=cn*8 +7;
     A(i,j)=-(720/(S(cn+2,1)-S(cn+1,1))^6);
     
     b(i,1)=0;
end
% 8n-4 consytaints done
A(8*n-3,3)=2/(S(2,1)-S(1,1))^2;
b(8*n-3,1)=0;
for i=3:8
A(8*n-2,(n-1)*8+i)=(i-1)*(i-2)/(S(n+1,1)-S(n,1))^2;
end
b(8*n-2,1)=0;
A(8*n-1,4)=6/(S(2,1)-S(1,1))^3;
b(8*n-1,1)=0;
for i=4:8
A(8*n,(n-1)*8+i)=(i-3)*(i-2)*(i-1)/(S(n+1,1)-S(n,1))^3;
end
b(8*n,1)=0;
alpha=A\b;

dt=0.1;
time=0;
no_s=round(S(end,1)/dt);
i=0;
sl=0;




%for y

A2=zeros(8*n,8*n);
b2=zeros(8*n,1);
for i=1:n
    A2(i,(i-1)*8+1)=1;
    b2(i,1)=waypoints0(2,i);
end
% n constraints done
for i=n:2*n-1
    t1=rem(i,n);
    for j=t1*8+1:t1*8+8
        A2(i+1,j)=1;
    end
    b2(i+1,1)=waypoints0(2,t1+2);
end
% 2n constraints done
A2(2*n+1,2)=1/(S(2,1)-S(1,1));
b2(2*n+1,1)=V1(2,1);%0;
for i=2:8
A2(2*n+2,(n-1)*8+i)=(i-1)/(S(n+1,1)-S(n,1));
end
b2(2*n+2,1)=V1(2,1);%0;
% 2n+2 constraints are done
cn=0; %for velocity
for i=2*n+3:3*n+1
    cn=cn+1;
    pw=2;
    for j=(cn-1)*8+2:(cn-1)*8+8
        A2(i,j)=(pw-1)/(S(cn+1,1)-S(cn,1));
        pw=pw+1;
    end
     j=cn*8 +2;
     A2(i,j)=-1/(S(cn+2,1)-S(cn+1,1));
     
     b2(i,1)=0;
end
%3n+1 constraints done

%for acceleration
cn=0; 
for i=3*n+2:4*n
    cn=cn+1;
    pw=3;
    for j=(cn-1)*8+3:(cn-1)*8+8
        A2(i,j)=(pw-1)*(pw-2)/((S(cn+1,1)-S(cn,1))^2);
        pw=pw+1;
    end
     j=cn*8 +3;
     A2(i,j)=-(2/(S(cn+2,1)-S(cn+1,1))^2);
     
     b2(i,1)=0;
end
% 4n constraints done
cn=0; 
for i=4*n+1:5*n-1
    cn=cn+1;
    pw=4;
    for j=(cn-1)*8+4:(cn-1)*8+8
        A2(i,j)=(pw-1)*(pw-2)*(pw-3)/((S(cn+1,1)-S(cn,1))^3);
        pw=pw+1;
    end
     j=cn*8 +4;
     A2(i,j)=-(6/(S(cn+2,1)-S(cn+1,1))^3);
     
     b2(i,1)=0;
end


cn=0; 
for i=5*n:6*n-2
    cn=cn+1;
    pw=5;
    for j=(cn-1)*8+5:(cn-1)*8+8
        A2(i,j)=(pw-1)*(pw-2)*(pw-3)*(pw-4)/((S(cn+1,1)-S(cn,1))^4);
        pw=pw+1;
    end
     j=cn*8 +5;
     A2(i,j)=-(24/(S(cn+2,1)-S(cn+1,1))^4);
     
     b2(i,1)=0;
end

cn=0; 
for i=6*n-1:7*n-3
    cn=cn+1;
    pw=6;
    for j=(cn-1)*8+6:(cn-1)*8+8
        A2(i,j)=(pw-1)*(pw-2)*(pw-3)*(pw-4)*(pw-5)/((S(cn+1,1)-S(cn,1))^5);
        pw=pw+1;
    end
     j=cn*8 +6;
     A2(i,j)=-(120/(S(cn+2,1)-S(cn+1,1))^5);
     
     b2(i,1)=0;
end

cn=0; 
for i=7*n-2:8*n-4
    cn=cn+1;
    pw=7;
    for j=(cn-1)*8+7:(cn-1)*8+8
        A2(i,j)=(pw-1)*(pw-2)*(pw-3)*(pw-4)*(pw-5)*(pw-6)/((S(cn+1,1)-S(cn,1))^6);
        pw=pw+1;
    end
     j=cn*8 +7;
     A2(i,j)=-(720/(S(cn+2,1)-S(cn+1,1))^6);
     
     b2(i,1)=0;
end
% 8n-4 consytaints done
A2(8*n-3,3)=2/(S(2,1)-S(1,1))^2;
b2(8*n-3,1)=0;
for i=3:8
A2(8*n-2,(n-1)*8+i)=(i-2)*(i-1)/(S(n+1,1)-S(n,1))^2;
end
b2(8*n-2,1)=0;
A2(8*n-1,4)=6/(S(2,1)-S(1,1))^3;
b2(8*n-1,1)=0;
for i=4:8
A2(8*n,(n-1)*8+i)=(i-3)*(i-2)*(i-1)/(S(n+1,1)-S(n,1))^3;
end
b2(8*n,1)=0;
alpha2=A2\b2;


   % for z
   
   A3=zeros(8*n,8*n);
b3=zeros(8*n,1);
for i=1:n
    A3(i,(i-1)*8+1)=1;
    b3(i,1)=waypoints0(3,i);
end
% n constraints done
for i=n:2*n-1
    t1=rem(i,n);
    for j=t1*8+1:t1*8+8
        A3(i+1,j)=1;
    end
    b3(i+1,1)=waypoints0(3,t1+2);
end
% 2n constraints done
A3(2*n+1,2)=1/(S(2,1)-S(1,1));
b3(2*n+1,1)=0;
for i=2:8
A3(2*n+2,(n-1)*8+i)=(i-1)/(S(n+1,1)-S(n,1));
end
b3(2*n+2,1)=0;
% 2n+2 constraints are done
cn=0; %for velocity
for i=2*n+3:3*n+1
    cn=cn+1;
    pw=2;
    for j=(cn-1)*8+2:(cn-1)*8+8
        A3(i,j)=(pw-1)/(S(cn+1,1)-S(cn,1));
        pw=pw+1;
    end
     j=cn*8 +2;
     A3(i,j)=-1/(S(cn+2,1)-S(cn+1,1));
     
     b3(i,1)=0;
end
%3n+1 constraints done

%for acceleration
cn=0; 
for i=3*n+2:4*n
    cn=cn+1;
    pw=3;
    for j=(cn-1)*8+3:(cn-1)*8+8
        A3(i,j)=(pw-1)*(pw-2)/((S(cn+1,1)-S(cn,1))^2);
        pw=pw+1;
    end
     j=cn*8 +3;
     A3(i,j)=-(2/(S(cn+2,1)-S(cn+1,1))^2);
     
     b3(i,1)=0;
end
% 4n constraints done
cn=0; 
for i=4*n+1:5*n-1
    cn=cn+1;
    pw=4;
    for j=(cn-1)*8+4:(cn-1)*8+8
        A3(i,j)=(pw-1)*(pw-2)*(pw-3)/((S(cn+1,1)-S(cn,1))^3);
        pw=pw+1;
    end
     j=cn*8 +4;
     A3(i,j)=-(6/(S(cn+2,1)-S(cn+1,1))^3);
     
     b3(i,1)=0;
end


cn=0; 
for i=5*n:6*n-2
    cn=cn+1;
    pw=5;
    for j=(cn-1)*8+5:(cn-1)*8+8
        A3(i,j)=(pw-1)*(pw-2)*(pw-3)*(pw-4)/((S(cn+1,1)-S(cn,1))^4);
        pw=pw+1;
    end
     j=cn*8 +5;
     A3(i,j)=-(24/(S(cn+2,1)-S(cn+1,1))^4);
     
     b3(i,1)=0;
end

cn=0; 
for i=6*n-1:7*n-3
    cn=cn+1;
    pw=6;
    for j=(cn-1)*8+6:(cn-1)*8+8
        A3(i,j)=(pw-1)*(pw-2)*(pw-3)*(pw-4)*(pw-5)/((S(cn+1,1)-S(cn,1))^5);
        pw=pw+1;
    end
     j=cn*8 +6;
     A3(i,j)=-(120/(S(cn+2,1)-S(cn+1,1))^5);
     
     b3(i,1)=0;
end

cn=0; 
for i=7*n-2:8*n-4
    cn=cn+1;
    pw=7;
    for j=(cn-1)*8+7:(cn-1)*8+8
        A3(i,j)=(pw-1)*(pw-2)*(pw-3)*(pw-4)*(pw-5)*(pw-6)/((S(cn+1,1)-S(cn,1))^6);
        pw=pw+1;
    end
     j=cn*8 +7;
     A3(i,j)=-(720/(S(cn+2,1)-S(cn+1,1))^6);
     
     b3(i,1)=0;
end
% 8n-4 consytaints done
A3(8*n-3,3)=2/(S(2,1)-S(1,1))^2;
b3(8*n-3,1)=0;
for i=3:8
A3(8*n-2,(n-1)*8+i)=(i-2)*(i-1)/(S(n+1,1)-S(n,1))^2;
end
b3(8*n-2,1)=0;
A3(8*n-1,4)=6/(S(2,1)-S(1,1))^3;
b3(8*n-1,1)=0;
for i=4:8
A3(8*n,(n-1)*8+i)=(i-3)*(i-2)*(i-1)/(S(n+1,1)-S(n,1))^3;
end
b3(8*n,1)=0;
alpha3=A3\b3; 
end


if nargin ==2 
    if length(state)~=0
for i=1:n
    if t>=S(i) && t<=S(i+1)
%         if t>14
%             pause
%         end
        %position
        sm=0;
        pw=0;
        sm2=0;
        sm3=0;
        for j=(i-1)*8+1:(i-1)*8+8
            %pw=rem(j,8)-1;
             sm=sm+ alpha(j,1)*((t-S(i,1))/(S(i+1,1)-S(i,1)))^pw;
             sm2=sm2+ alpha2(j,1)*((t-S(i,1))/(S(i+1,1)-S(i,1)))^pw;
             sm3=sm3+ alpha3(j,1)*((t-S(i,1))/(S(i+1,1)-S(i,1)))^pw;
             pw=pw+1;
        end
        desired_state.pos(1,1)=sm;
        desired_state.pos(2,1)=sm2;
        desired_state.pos(3,1)=sm3;
        %velocity
        
         sm=0;
        pw=0;
        sm2=0;
        sm3=0;
        for j=(i-1)*8+2:(i-1)*8+8
            %pw=rem(j,8)-1;
             sm=sm+ (alpha(j,1)*((t-S(i,1))/(S(i+1,1)-S(i,1)))^pw)/(S(i+1,1)-S(i,1))*(pw+1);
             sm2=sm2+ (alpha2(j,1)*((t-S(i,1))/(S(i+1,1)-S(i,1)))^pw)/(S(i+1,1)-S(i,1))*(pw+1);
             sm3=sm3+ (alpha3(j,1)*((t-S(i,1))/(S(i+1,1)-S(i,1)))^pw)/(S(i+1,1)-S(i,1))*(pw+1);
             pw=pw+1;
        end
        desired_state.vel(1,1)=sm;
        desired_state.vel(2,1)=sm2;
        desired_state.vel(3,1)=sm3;
        
        %acceleration
         sm=0;
        pw=0;
        sm2=0;
        sm3=0;
        for j=(i-1)*8+3:(i-1)*8+8
            %pw=rem(j,8)-1;
             sm=sm+ (alpha(j,1)*((t-S(i,1))/(S(i+1,1)-S(i,1)))^pw)/((S(i+1,1)-S(i,1))^2)*((pw+1)*(pw+2));
             sm2=sm2+ (alpha2(j,1)*((t-S(i,1))/(S(i+1,1)-S(i,1)))^pw)/((S(i+1,1)-S(i,1))^2)*((pw+1)*(pw+2));
             sm3=sm3+ (alpha3(j,1)*((t-S(i,1))/(S(i+1,1)-S(i,1)))^pw)/((S(i+1,1)-S(i,1))^2)*((pw+1)*(pw+2));
             pw=pw+1;
        end
        desired_state.acc(1,1)=sm;
        desired_state.acc(2,1)=sm2;
        desired_state.acc(3,1)=sm3;
        desired_state.yaw=atan2(desired_state.vel(2,1),desired_state.vel(1,1));
        desired_state.yawdot=atan2(desired_state.acc(2,1),desired_state.acc(1,1));
    end
end
    end
end

if length(state)==0
    if t==0
    desired_state.pos=waypoints0(:,1);
    desired_state.vel=((waypoints0(:,2)-waypoints0(:,1))/(traj_time(2)-traj_time(1)));%[0,0,0];
    desired_state.acc=[0,0,0];
    desired_state.yaw=atan2(desired_state.vel(2,1),desired_state.vel(1,1));%0;
    desired_state.yawdot=0;
    end
    if t==Inf
        desired_state.pos=waypoints0(:,end);%[4;0;0];
    desired_state.vel=(waypoints0(:,2)-waypoints0(:,1))/(traj_time(2)-traj_time(1));%[0,0,0];
    desired_state.acc=[0,0,0];
    desired_state.yaw=atan2(desired_state.vel(2,1),desired_state.vel(1,1));%0;
    desired_state.yawdot=0;
    end
end

end
    

