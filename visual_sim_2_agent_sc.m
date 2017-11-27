function[total_energy ,C_eq, C_ineq]= visual_sim_2_agent_sc(X1,varargin)
% this function is for speed change
V1 = varargin{1,1};
V2 = varargin{1,2};
Pa_i = varargin{1,3};
Pb_i = varargin{1,4};
Pa_f = varargin{1,5};
Pb_f = varargin{1,6};
V_max = varargin{1,7};
% t_trans=varargin{1,8};
method=varargin{1,8};
t_col=varargin{1,9};

if norm(V1)>norm(V2)
    delta_va = X1(2);
    delta_vb = -X1(2);
else
    delta_va = -X1(2);
    delta_vb = X1(2);
end

%% design variables
t2=X1(1);
speed_new_a=norm(V1)+delta_va;
speed_new_b= norm(V2)+delta_vb;
Vnew1_a=speed_new.*V1./(norm(V1));
Vnew1_b=speed_new_b.*V2./(norm(V2));
%% agent 1
t1=0;
P1=Pa_i;
t2=X1(1); %time action starts
P2= Pa_i + V1.*t2; %point at which agent a changes speed
t3=t_col;
P3=P2+Vnew1_a.*(t3-t2);

P4=P2+V1.*(2*(t3-t2));
Vnew2_a=(P4-P3)./(t3-t2);
speednew=norm(Vnew2_a);

t4=t3+(t3-t2);
t5=10;
P5=P4+V1.*(t5-t4);

C_eq=[]; C_ineq=[];
% C_eq(1)=abs(speed-speed_rqd)/speed_rqd;

%% Trajectory 
trajhandle = @traj_generator;
waypoints=[[P1' 0]; [P2' 0]; [P3' 0]; [P4' 0];[P5' 0]]';
time=[t1 t2 t3 t4 t5]';

trajhandle([],[],waypoints,time,t5,[V1;0]);
controlhandle = @controller;
%%
%% agent 2
t1=0;
P1=Pb_i;
t2=X1(1); %time action starts
P2= Pb_i + V2.*t2; %point at which agent a changes speed
t3=t_col;
P3=P2+Vnew1_b.*(t3-t2);

P4=P2+V2.*(2*(t3-t2));
Vnew2_b=(P4-P3)./(t3-t2);
speednew=norm(Vnew2_b);

t4=t3+(t3-t2);
t5=10;
P5=P4+V2.*(t5-t4);
C_eq=[]; C_ineq=[];
% C_eq(1)=abs(speed-speed_rqd)/speed_rqd;
C_eq=[];
%% Trajectory 
trajhandle2 = @traj_generator2;
waypoints2=[[P1' 0]; [P2' 0]; [P3' 0]; [P4' 0];[P5' 0]]';
trajhandle2([],[],waypoints2,time,t5,[V2;0]);
controlhandle2 = @controller2;


%%
% [total_energy,C_ineq] = simulation(trajhandle, controlhandle,Pb_i,V2,t5);
[total_energy,C_ineq] = simulation_2quads(trajhandle, controlhandle,trajhandle2, controlhandle2,Pb_i,V2,t5);
end
