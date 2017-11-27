function [X fx] = SC(V1,V2,Pa_i,Pb_i,Pa_f,Pb_f,V_max,t_det)
% addpath('MDPSO_MATLAB_2016')

nvars=2;

ub = min((V_max-max(norm(V1),norm(V2))),min(norm(V1),norm(V2)))


Lb=[0;0];
Ub=[.6*t_det;ub];

cons=2;
dvvec=[0,0];
method=2;
vartype=zeros(2,2);
func_param=[nvars,2,0,1,3,cons,1];
algo_inputs=MDPSO_input_param(func_param,'Itermax',35,'InitpopGen','LHS');
run_options=MDPSO_run_options('display','OFF','plotconv','OFF','useparallel' , 'on');

[optimum, exit_flag] = MDPSO('visual_sim_2_agent_sc',func_param,1,vartype,[Lb Ub],dvvec,algo_inputs,[],run_options,V1, V2, Pa_i, Pb_i, Pa_f, Pb_f, V_max,method,t_det);
X=optimum.var;
fx=optimum.obj;
end