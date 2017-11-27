function[total_energy ,C_eq, C_ineq]= visual_sim_2_agent(X1,X2,varargin)
% this function is for direction change

V1 = varargin{1,1};
V2 = varargin{1,2};
Pa_i = varargin{1,3};
Pb_i = varargin{1,4};
Pa_f = varargin{1,5};
Pb_f = varargin{1,6};
V_max = varargin{1,7};
%t_trans=varargin{1,8};
method=varargin{1,8};
t_col=varargin{1,9};
% X2=X1;
%% design variables
t2=X1(1);
delta_theta1=X1(2);
speed_rqd1=norm(V1)/cos(delta_theta1);
speed1=min(speed_rqd1,V_max);
t1=0;
P1=Pa_i;
t2=X1(1); %time action starts
P2= Pa_i + V1.*t2; %point at which agent a changes speed
t3=t_col;
heading1=atan2(V1(2,1),V1(1,1));% calculating the heading angle
nw_heading1=rem(heading1+delta_theta1,2*pi);
Va_new=speed1*[cos(nw_heading1);sin(nw_heading1)];
P3=P2+Va_new.*(t3-t2);
P4=P2+V1.*(2*(t3-t2));
delta_t=norm(P4-P3)/speed1;
t4=t3+t3-t2;
t5=10;
P5=Pa_i+V1.*t5;
C_eq=[]; C_ineq=[];
C_eq(1)=abs(speed1-speed_rqd1)/speed_rqd1;
Vnew2=(P4-P3)./(t4-t3);
% [waypoints time1 waypoints2 time2]=inter_wp_time(Xn,V1, V2, tc, Pa_i, Pb_i, Pa_f, Pb_f, V_max,tc1,t_trans,method,t_col);
%% Trajectory 
trajhandle = @traj_generator;
 waypoints=[[P1' 0]; [P2' 0]; [P3' 0]; [P4' 0];[P5' 0]]';% waypoints for A
 time=[t1 t2 t3 t4 t5]';

 trajhandle([],[],waypoints,time,t5,[V1;0]);% trajectory for A
%%
t2=X2(1);
delta_theta=X2(2);
speed_rqd=norm(V2)/cos(delta_theta);
speed=min(speed_rqd,V_max);
t1=0;
P1=Pb_i;
t2=X2(1); %time action starts
P2= Pb_i + V2.*t2; %point at which agent a changes speed
t3=t_col;
heading=atan2(V2(2,1),V2(1,1)); % calculating the heading angle
nw_heading=rem(heading+delta_theta,2*pi);
Vb_new=speed*[cos(nw_heading);sin(nw_heading)];
P3=P2+Vb_new.*(t3-t2);
P4=P2+V2.*(2*(t3-t2));
delta_t=norm(P4-P3)/speed;
t4=t3+t3-t2;
t5=10;
P5=Pb_i+V2.*t5;
C_eq=[]; C_ineq=[];
C_eq(1)=abs(speed-speed_rqd)/speed_rqd;
Vnew2=(P4-P3)./(t4-t3);
% [waypoints time1 waypoints2 time2]=inter_wp_time(Xn,V1, V2, tc, Pa_i, Pb_i, Pa_f, Pb_f, V_max,tc1,t_trans,method,t_col);
%% Trajectory 
trajhandle2 = @traj_generator2;
waypoints2=[[P1' 0]; [P2' 0]; [P3' 0]; [P4' 0];[P5' 0]]';% waypoints for B
time2=[t1 t2 t3 t4 t5]';

%%

trajhandle2([],[],waypoints2,time2,t5,[V2;0]); % trajectory for B

%% controller
controlhandle = @controller; % controller for A
controlhandle2 = @controller2; % controller for B
% use simulation_2quads_visual to see the flight of the two UAVs
[total_energy,C_ineq] = simulation_2quads_visual(trajhandle, controlhandle,trajhandle2, controlhandle2,Pb_i,V2,t5);
end
