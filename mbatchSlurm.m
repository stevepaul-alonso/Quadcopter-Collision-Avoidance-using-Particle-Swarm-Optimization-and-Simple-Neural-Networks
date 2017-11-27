function f=mbatch()
%% Submission script for using the MDCS at CCR, API-2
startupSlurm %need this to load API-2 settings

StorageLocation='/gpfs/scratch/stevepaul_trial2'; %<== enter YOUR dir
ppn=12;
time='025:00:00';
email='stevepau@buffalo.edu';  %%<== enter YOUR e-mail

%%Don't modify this line%%
set(u2,'JobStorageLocation',StorageLocation,'CommunicatingSubmitFcn', ...
    {@communicatingSubmitFcnSlurm,ppn,time,email});

%% for spmd code
%  pjob = createCommunicatingJob(u2,'Type', 'SPMD');
%  pjob.NumWorkersRange = [1 24]; %%Set Maximum Workers (cores);

%% for code with parfor loops or functions w/ built-in parallelism (ex. optimization toolbox stuff)
pjob = createCommunicatingJob(u2);
pjob.NumWorkersRange = [1 120]; %%Set Maximum Workers (cores)

%% Example for submitting a function (piMC.m) w/ 1 output and 1 input:   
set(pjob, 'AttachedFiles',{'main.m','DC.m','SC.m','obj_fn.m','simulation.m','controller.m','controller2.m','traj_generator.m','traj_generator2.m','transition_begin_time.m','time_to_min_dist_15.m','time_to_min_dist.m','terminate_check.m','sys_params.m','stateToQd.m','solution_comparison.m','RPYtoRot_ZXY.m','RotToRPY_ZXY.m','RotToQuat.m','QuatToRot.m','QuadPlot.m','quadEOM.m','quad_pos.m','Power.m','plot_state.m','plot_convergence.m','minimum_sep.m','MDPSO_run_options.m','MDPSO_input_param.m','MDPSO_control_param.m','MDPSO.m','inter_wp_time.m','init_state.m','function_wrapper.m','diversity_eval.m','discrete_update.m','create_sample_points_new.m','col_det2.m'}); %list files needed at runtime                              
createTask(pjob,@main,1,{5e6});  %Monte-Carlo pi-calculation

%% Example for a function (Julia.m) w/ 1 output and 3 inputs
%set(pjob, 'AttachedFiles',{'Julia.m'}); % list files needed at runtime
%createTask(pjob,@Julia,1,{1000, -0.8, 0.156});  %Julia set fractal spmd code

%%Another example: diffusion/automata code with send/recieve calls
%set(pjob, 'AttachedFiles', {'flakeMPI.m'});
%createTask(pjob,@flakeMPI,1,{768}); % input (size) must be divisible by # of cores

submit(pjob);
f=pjob;
