function[total_energy ,C_eq, C_ineq]= obj_fn(Xn,varargin)

V1 = varargin{1,1};
V2 = varargin{1,2};
Pa_i = varargin{1,3};
Pb_i = varargin{1,4};
Pa_f = varargin{1,5};
Pb_f = varargin{1,6};
V_max = varargin{1,7};
 t_trans=varargin{1,8};
method=varargin{1,8};
t_col=varargin{1,9};

%% design variables
t2=Xn(1);
delta_theta=Xn(2);
speed_rqd=norm(V1)/cos(delta_theta);
speed=min(speed_rqd,V_max);
t1=0;
P1=Pa_i;
t2=Xn(1); %time action starts
P2= Pa_i + V1.*t2; %point at which agent a changes speed
t3=t_col;
heading=atan2(V1(2,1),V1(1,1));
nw_heading=rem(heading+delta_theta,2*pi);
Va_new=speed*[cos(nw_heading);sin(nw_heading)];
P3=P2+Va_new.*(t3-t2);
P4=P2+V1.*(2*(t3-t2));
delta_t=norm(P4-P3)/speed;
t4=t3+delta_t;
t5=ceil(t4+3);
P5=Pa_i+V1.*t5;
t6=(t3+t2)/2;
P6=P2+Va_new.*(t6-t2);
C_eq=[]; C_ineq=[];
C_eq(1)=abs(speed-speed_rqd)/speed_rqd;
Vnew2=(P4-P3)./(t4-t3);
t7=(t4+t3)/2;
P7=P3+Vnew2.*(t7-t3);




% [waypoints time1 waypoints2 time2]=inter_wp_time(Xn,V1, V2, tc, Pa_i, Pb_i, Pa_f, Pb_f, V_max,tc1,t_trans,method,t_col);
%% Trajectory 
trajhandle = @traj_generator;
% waypoints=[[P1' 0]; [P2' 0]; [P6' 0]; [P3' 0]; [P7' 0]; [P4' 0];[P5' 0]]';
% time=[t1 t2 t6 t3 t7 t4 t5]';
dt=(t5-t1)/15;
time=[t1:dt:t5]';
P=[P1;0];
for i=1:length(time)
    if time(i)<=t2
        V=[V1;0];
    elseif time(i)>t2 && time(i)<=t3
        V=[Va_new;0];
    elseif time(i)>t3 && time(i)<=t4
        V=[Vnew2;0];
    else
        V=[V1;0];
    end
    P=P+V.*dt;
    waypoints(:,i)=P;
end
trajhandle([],[],waypoints,time,t5,[V1;0]);

%% controller
controlhandle = @controller;

[total_energy,C_ineq] = simulation(trajhandle, controlhandle,Pb_i,V2,t5);
end
