function dt=org_path(Pa_i,V1,Pb_i,V2)
% V1 = varargin{1,1};
% V2 = varargin{1,2};
% Pa_i = varargin{1,3};
% Pb_i = varargin{1,4};
% Pa_f = varargin{1,5};
% Pb_f = varargin{1,6};
% V_max = varargin{1,7};
% % t_trans=varargin{1,8};
% method=varargin{1,8};
% t_col=varargin{1,9};
V1=V1./2;
P1=Pa_i;
t1=0;
t2=20/4
t3=20/2
t4=20/4*3;
t5=20;
P2=V1.*t2
P3=V1.*t3
P4=V1.*t4
P5=V1.*t5
trajhandle = @traj_generator;
waypoints=[[P1' 0]; [P2' 0]; [P3' 0]; [P4' 0];[P5' 0]]';
time=[t1 t2 t3 t4 t5]';
trajhandle([],[],waypoints,time,t5,[V1;0]);

%% controller
controlhandle = @controller;

[total_energy,C_ineq] = simulation_2(trajhandle, controlhandle,Pb_i,V2,t5);
dt=1;
end

