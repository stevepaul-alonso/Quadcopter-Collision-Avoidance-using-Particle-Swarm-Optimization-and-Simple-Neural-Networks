function[total_energy]= original_motion(samples,vmax,x1,x2)
V1 = samples.Va;
V2 = samples.Vb;
Pa_i = samples.Pa;
Pb_i = samples.Pb;
%Pa_f = varargin{1,5};
%Pb_f = varargin{1,6};
V_max = vmax;
%t_trans=varargin{1,8};
method=1;
t_col=5;
% X2=X1;
%% design variables
t2=x1;
delta_theta1=0.0;
speed_rqd1=norm(V1)/cos(delta_theta1);
speed1=min(speed_rqd1,V_max);
t1=0;
P1=Pa_i;
t2=x1; %time action starts
P2= Pa_i + V1.*t2; %point at which agent a changes speed
t3=t_col;
heading1=atan2(V1(2,1),V1(1,1));
nw_heading1=rem(heading1+delta_theta1,2*pi);
Va_new=speed1*[cos(nw_heading1);sin(nw_heading1)];
P3=P2+Va_new.*(t3-t2);
P4=P2+V1.*(2*(t3-t2));
delta_t=norm(P4-P3)/speed1;
t4=t3+delta_t;
t5=ceil(t4+3);
P5=Pa_i+V1.*t5;
t6=(t3+t2)/2;
P6=P2+Va_new.*(t6-t2);
C_eq=[]; C_ineq=[];
C_eq(1)=abs(speed1-speed_rqd1)/speed_rqd1;
Vnew2=(P4-P3)./(t4-t3);
t7=(t4+t3)/2;
P7=P3+Vnew2.*(t7-t3);
t51=t5;
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
        V=[V1;0];
    elseif time(i)>t3 && time(i)<=t4
        V=[V1;0];
    else
        V=[V1;0];
    end
    P=P+V.*dt;
    waypoints(:,i)=P;
end
trajhandle([],[],waypoints,time,t5,[V1;0]);
%%
t2=x2;
delta_theta=0.0;
speed_rqd=norm(V2)/cos(delta_theta);
speed=min(speed_rqd,V_max);
t1=0;
P1=Pb_i;
t2=x2; %time action starts
P2= Pb_i + V2.*t2; %point at which agent a changes speed
t3=t_col;
heading=atan2(V2(2,1),V2(1,1));
nw_heading=rem(heading+delta_theta,2*pi);
Vb_new=speed*[cos(nw_heading);sin(nw_heading)];
P3=P2+Vb_new.*(t3-t2);
P4=P2+V2.*(2*(t3-t2));
delta_t=norm(P4-P3)/speed;
t4=t3+delta_t;
t5=ceil(t4+3);
P5=Pb_i+V2.*t5;
t6=(t3+t2)/2;
P6=P2+Vb_new.*(t6-t2);
C_eq=[]; C_ineq=[];
C_eq(1)=abs(speed-speed_rqd)/speed_rqd;
Vnew2=(P4-P3)./(t4-t3);
t7=(t4+t3)/2;
P7=P3+Vnew2.*(t7-t3);
t52=t5;
% [waypoints time1 waypoints2 time2]=inter_wp_time(Xn,V1, V2, tc, Pa_i, Pb_i, Pa_f, Pb_f, V_max,tc1,t_trans,method,t_col);
%% Trajectory 
trajhandle2 = @traj_generator2;
% waypoints=[[P1' 0]; [P2' 0]; [P6' 0]; [P3' 0]; [P7' 0]; [P4' 0];[P5' 0]]';
% time=[t1 t2 t6 t3 t7 t4 t5]';
dt=(t5-t1)/15;
time2=[t1:dt:t5]';
P=[P1;0];
for i=1:length(time2)
    if time2(i)<=t2
        V=[V2;0];
    elseif time2(i)>t2 && time2(i)<=t3
        V=[V2;0];
    elseif time2(i)>t3 && time2(i)<=t4
        V=[V2;0];
    else
        V=[V2;0];
    end
    P=P+V.*dt;
    waypoints2(:,i)=P;
end

%%

trajhandle2([],[],waypoints2,time2,t5,[V2;0]);

%% controller
controlhandle = @controller;
controlhandle2 = @controller2;
t5=min(t51,t52);
[total_energy,C_ineq] = simulation_2quads(trajhandle, controlhandle,trajhandle2, controlhandle2,Pb_i,V2,t5);
end
