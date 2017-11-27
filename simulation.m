% function [t_out, s_out] = simulation_2quads(trajhandle, controlhandle,trajhandle2, controlhandle2)
function [total_energy,cons] = simulation(trajhandle, controlhandle,Pb_i,Vb,t5)
cn=1;
tic
cons=[];%zeros(2,1);
% addpath('utils');

% real-time
% real_time = true;

% max time
max_time = t5;

% parameters for simulation
params = sys_params;

%% Figure
% disp('Initializing figures...');
% h_fig = figure;
% h_3d = gca;
% axis equal
% grid on
% view(3);
% xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
% quadcolors = lines(1);
% 
% set(gcf,'Renderer','OpenGL')

%%Initial conditions
% disp('Setting initial conditions...');
tstep    = 0.01; % this determines the time step at which the solution is given
cstep    = .05; % image capture time interval
max_iter = max_time/cstep; % max iteration
nstep    = cstep/tstep;
time     = 0; % current time
err = []; % runtime errors

% Get start and stop position for agent 1
des_start = trajhandle(0, []);
des_stop  = trajhandle(inf, []);
stop_pos  = des_stop.pos;
chk=sum(isnan(des_start.pos))+sum(isnan(des_start.vel))+sum(isnan(des_start.yaw));
if chk>0
    total_energy=100000;
      C_eq=[];
      cons(1)=5000;
      cons(2)=3;
      return
end
chk2=des_start.pos+des_start.vel+des_start.yaw;
if sum(chk2)==Inf
    total_energy=100000;
      C_eq=[];
      cons(1)=5000;
      cons(2)=3;
      return
end
    
x0    = init_state(des_start.pos, des_start.vel,des_start.yaw);
xtraj = zeros(max_iter*nstep, length(x0));
ttraj = zeros(max_iter*nstep, 1);
energytraj=zeros(max_iter*nstep, 4);

%% Get start and stop position for agent 2
% des_start2 = trajhandle2(0, []);
% des_stop2  = trajhandle2(inf, []);
% stop_pos2  = des_stop2.pos;
% x02    = init_state(des_start2.pos,des_start2.vel, des_start2.yaw);
% xtraj2 = zeros(max_iter*nstep, length(x02));
% ttraj2 = zeros(max_iter*nstep, 1);
% energytraj2=zeros(max_iter*nstep, 4);


x  = x0;        % state
% x2 = x02;


pos_tol = 0.01;
vel_tol = 0.01;

%% Simulation
% disp('Simulation Running....');
% Main loop

for iter = 1:max_iter-1

    timeint = time:tstep:time+cstep;

%     tic;

    % Initialize quad plot
    
    if iter == 1
        
%         QP = QuadPlot(1, x0, 0.1, 0.04, quadcolors(1,:), max_iter, h_3d);
%         QP2 = QuadPlot(2, x02, 0.1, 0.04, quadcolors(1,:), max_iter, h_3d);
        current_state = stateToQd(x);
        desired_state = trajhandle(time, current_state);
%         current_state2 = stateToQd(x2);
%         desired_state2 = trajhandle2(time, current_state2);
%         QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time);
%         QP2.UpdateQuadPlot(x2, [desired_state2.pos; desired_state2.vel], time);
%         h_title = title(sprintf('time: %4.2f',  time));
        x(14:17,1)=zeros(4,1);
%         x2(14:17,1)=zeros(4,1);
    
    end
   
    % Run simulation
%      options = odeset('RelTol',1e-7);
    [tsave, xsave] = ode23(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params), timeint, x);
%    [tsave2, xsave2] = ode23(@(t,s) quadEOM(t, s, controlhandle2, trajhandle2, params), timeint, x2);
    x    = xsave(end,:)';
%     x2    = xsave2(end,:)';
    [s1,s2]=size(xsave);
%     [s12,s22]=size(xsave2);
    if s1~=6 
        cn=0;
%         display('breaking')
        break;
    end

    % Save to traj for agent1
    xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:nstep,1:13);
    ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
    energytraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,14:17);
    
      % Save to traj for agent2
%     xtraj2((iter-1)*nstep+1:iter*nstep,:) = xsave2(1:end-1,1:13);
%     ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
%     energytraj2((iter-1)*nstep+1:iter*nstep,:) = xsave2(1:end-1,14:17);

    % Update quad plot for agent 1
    current_state = stateToQd(x(1:13,1));
    desired_state = trajhandle(time + cstep, current_state);
    
    
    % Update quad plot for agent 2
%     current_state2 = stateToQd(x2(1:13,1));
%     desired_state2 = trajhandle2(time + cstep, current_state2);
%     
%     QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time + cstep);
%     QP2.UpdateQuadPlot(x2, [desired_state2.pos; desired_state2.vel], time + cstep);
%     set(h_title, 'String', sprintf('time: %4.2f',time + cstep))

    time = time + cstep; % Update simulation time


end
  if cn==0
      total_energy=100000;
      C_eq=[];
      cons(1)=5000;
      cons(2)=3;
      return
  end

%% Results
% Truncate xtraj and ttraj
xtraj = xtraj(1:iter*nstep,:);
ttraj = ttraj(1:iter*nstep);
% xtraj2 = xtraj2(1:iter*nstep,:);
% ttraj = ttraj(1:iter*nstep);
xtraj2=Pb_i'+ttraj.*Vb';  %% the change for new edition
for i=1:iter*nstep
sep(i,1)=norm(xtraj(i,1:2)-xtraj2(i,1:2));
end
min_sep=min(sep);
% % Truncate saved variables
% QP.TruncateHist();
% QP2.TruncateHist();

% % % % Plot position
% h_pos = figure('Name', ['Position']);
% plot_state(h_pos, QP2.state_hist(1:3,:), QP2.time_hist, 'pos', 'vic');
% plot_state(h_pos, QP2.state_des_hist(1:3,:), QP2.time_hist, 'pos', 'des');
% % % Plot velocity
% h_vel = figure('Name', ['Velocity']);
% plot_state(h_vel, QP2.state_hist(4:6,:), QP2.time_hist, 'vel', 'vic');
% plot_state(h_vel, QP2.state_des_hist(4:6,:), QP2.time_hist, 'vel', 'des');

if(~isempty(err))
    error(err);
end

% disp('finished.')

t_out = ttraj;
s_out = xtraj;
total_energy=sum(energytraj(iter*nstep-5,:))
cons(2)=(3-min_sep);
% C_eq=[];
toc
end
