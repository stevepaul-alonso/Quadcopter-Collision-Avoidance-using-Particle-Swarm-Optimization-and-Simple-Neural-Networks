function [X fx] = DC(V1,V2,Pa_i,Pb_i,Pa_f,Pb_f,V_max,t_det)

%altering the speed of both the agents

% V_max=V_max/2^.5;
nvars=2; % number of variables
% vmu=max(norm(V1), norm(V2));
% v_u=V_max-vmu;
% v_l=min(norm(V1), norm(V2));
% v_ub=min(v_l,v_u);
speed=norm(V1);
Lb=[0.1;0];  % upper and lower bounds
Ub=[.6*t_det;pi/6];
% [X fx]=particleswarm(@obj3,nvars,Lb,Ub);
% [X fx]=fmincon(@obj3,[.05;v_ub],[],[],[],[],Lb,Ub,@cons_case3);
% opts=gaoptimset('PopulationSize',50,'Generations',150,'Display','off','UseParallel',true);
% [X fx]=ga(@simulation3,nvars,[],[],[],[],Lb,Ub,@cons_case3);
cons=2;
dvvec=[0,0];
method=2;
vartype=zeros(2,2);
func_param=[nvars,2,0,1,2,cons,0];
algo_inputs=MDPSO_input_param(func_param,'Itermax',50,'InitpopGen','LHS','IterNoChange',20);
run_options=MDPSO_run_options('display','OFF','plotconv','OFF','useparallel' , 'on');

[optimum, exit_flag] = MDPSO('visual_sim_2_agent',func_param,1,vartype,[Lb Ub],dvvec,algo_inputs,[],run_options,V1, V2, Pa_i, Pb_i, Pa_f, Pb_f, V_max,method,t_det);
X=optimum.var;
fx=optimum.obj
end