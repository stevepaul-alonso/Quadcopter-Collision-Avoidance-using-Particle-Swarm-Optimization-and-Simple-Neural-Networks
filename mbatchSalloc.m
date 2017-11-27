function f=mbatch()
%% Submission script for using the MDCS at CCR, API-2
startupSlurm %need this to load API-2 settings

StorageLocation='/panasas/scratch/usernameMatlabData'; %<== enter YOUR dir
ppn=12;
time='01:00:00';
email='username@buffalo.edu';  %%<== enter YOUR e-mail

%%Don't modify this line%%
set(u2,'JobStorageLocation',StorageLocation,'CommunicatingSubmitFcn', ...
    {@communicatingSubmitFcnSalloc,ppn,time,email});

%% for spmd code
 pjob = createCommunicatingJob(u2,'Type', 'SPMD');
 pjob.NumWorkersRange = [1 24]; %%Set Maximum Workers (cores);

%% for code with parfor loops or functions w/ built-in parallelism (ex. optimization toolbox stuff)
%pjob = createCommunicatingJob(u2);
%pjob.NumWorkersRange = [1 16]; %%Set Maximum Workers (cores)

%% Example for submitting a function (piMC.m) w/ 1 output and 1 input:   
set(pjob, 'AttachedFiles',{'piMC.m'}); %list files needed at runtime                              
createTask(pjob,@piMC,1,{5e6});  %Monte-Carlo pi-calculation

%% Example for a function (Julia.m) w/ 1 output and 3 inputs
%set(pjob, 'AttachedFiles',{'Julia.m'}); % list files needed at runtime
%createTask(pjob,@Julia,1,{1000, -0.8, 0.156});  %Julia set fractal spmd code

%%Another example: diffusion/automata code with send/recieve calls
%set(pjob, 'AttachedFiles', {'flakeMPI.m'});
%createTask(pjob,@flakeMPI,1,{768}); % input (size) must be divisible by # of cores

submit(pjob);
f=pjob;
