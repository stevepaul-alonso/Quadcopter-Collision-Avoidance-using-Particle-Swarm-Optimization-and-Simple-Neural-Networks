function [total_energy,cons] = simulation_2quads(trajhandle, controlhandle,trajhandle2, controlhandle2,Pb_i,Vb,t5)

cn=1;
tic
time_cap=[];
cons=[];%zeros(2,1);
% addpath('utils');
des_pos=[];
des_pos2=[];


% max time
max_time = t5;

% parameters for simulation
params = sys_params;



%%Initial conditions
% disp('Setting initial conditions...');
tstep    = 0.01; % this determines the time step at which the solution is given
cstep    = .05; % image capture time interval
max_iter = max_time/cstep; % max iteration
nstep    = cstep/tstep;
time     = 0; % current time
err = []; % runtime errors

% Get initial and final and stop position for agent 1
des_start = trajhandle(0, []);
des_stop  = trajhandle(inf, []);
stop_pos  = des_stop.pos;
x0    = init_state(des_start.pos, des_start.vel,des_start.yaw);
xtraj = zeros(max_iter*nstep, length(x0));
ttraj = zeros(max_iter*nstep, 1);
energytraj=zeros(max_iter*nstep, 4);

%% Get initial and final position for agent 2
des_start2 = trajhandle2(0, []);
des_stop2  = trajhandle2(inf, []);
stop_pos2  = des_stop2.pos;
x02    = init_state(des_start2.pos,des_start2.vel, des_start2.yaw);
xtraj2 = zeros(max_iter*nstep, length(x02));
ttraj2 = zeros(max_iter*nstep, 1);
energytraj2=zeros(max_iter*nstep, 4);


x  = x0;        % state
x2 = x02;


pos_tol = 0.01;
vel_tol = 0.01;

%% Simulation


for iter = 1:max_iter-1

    timeint = time:tstep:time+cstep;

%     tic;

    
    
    if iter == 1
        % initialise the current and desired state for the first iteration

        current_state = stateToQd(x);
        desired_state = trajhandle(time, current_state);
        current_state2 = stateToQd(x2);
        desired_state2 = trajhandle2(time, current_state2);

        x(14:17,1)=zeros(4,1);
        x2(14:17,1)=zeros(4,1);
    
    end
   
    % Run simulation
    
    [tsave, xsave] = ode23(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params), timeint, x);
   [tsave2, xsave2] = ode23(@(t,s) quadEOM(t, s, controlhandle2, trajhandle2, params), timeint, x2);
    x    = xsave(end,:)';
    x2    = xsave2(end,:)';
    [s1,s2]=size(xsave);
    [s12,s22]=size(xsave2);
    if s1~=6 || s12~=6
        cn=0;
        % if the flight of a UAV becomes unstable then the length of xsave
        % will not be 6
%         display('breaking')
%         break;
    end

    % Save to traj for agent1
    xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:nstep,1:13);
    ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
    energytraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,14:17);
    
      % Save to traj for agent2
    xtraj2((iter-1)*nstep+1:iter*nstep,:) = xsave2(1:end-1,1:13);
%     ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
    energytraj2((iter-1)*nstep+1:iter*nstep,:) = xsave2(1:end-1,14:17);

    % Update quad plot for agent 1
    current_state = stateToQd(x(1:13,1));
    desired_state = trajhandle(time + cstep, current_state);
    
    
    % Update quad plot for agent 2
    current_state2 = stateToQd(x2(1:13,1));
    desired_state2 = trajhandle2(time + cstep, current_state2);
%     

des_pos=[des_pos;desired_state.pos'];
des_pos2=[des_pos2;desired_state2.pos'];
time_cap=[time_cap;time];


    time = time + cstep; % Update simulation time


end
  if cn==0
      % if the flight is unstable add a huge penalty to the energy
      total_energy=10000;
      C_eq=[];
      cons(1)=5000;
      cons(2)=3;
%       return
  end

%% Results
% Truncate xtraj and ttraj
xtraj = xtraj(1:iter*nstep,:);
ttraj = ttraj(1:iter*nstep);
xtraj2 = xtraj2(1:iter*nstep,:);
% ttraj = ttraj(1:iter*nstep);
xtraj2=Pb_i'+time_cap.*Vb';  %% the change for new edition
for i=1:length(xtraj2)
sep(i,1)=norm(des_pos(i,1:2)-des_pos2(i,1:2));
end
min_sep=min(sep);




if(~isempty(err))
    error(err);
end


t_out = ttraj;
s_out = xtraj;
total_energy=sum(energytraj(iter*nstep-5,:))+sum(energytraj2(iter*nstep-5,:));
cons(2)=(3-min_sep);
% C_eq=[];
toc
% close all
end
