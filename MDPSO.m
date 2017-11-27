%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mixed-Discrete Particle Swarm Optimization (MDPSO)
% Date: 09/12/2014
% Author: Souma Chowdhury
% Copyright: By Souma Chowdhury, 2014
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [globalbest, exit_flag] = MDPSO(func, func_param, max_min, vartype, varlim, dvvec, algo_inputs, algo_control, run_options, varargin)

%%%%%%% Problem Input Parameters %%%%%%%%
% func: Objective function (expected format: [f,h,g] = objfun(x))
%
% func_param = [Nv, Nvc, Nvd, Nobj, Nc, Ncie, Nce]
% Nv:      # Variables
% Nvc:     # Continuous Variables
% Nvd:     # Discrete Variables
% Nobj:    # Objectives
% Nc:      # Constraints
% Ncie:    # Inequality Constraints
% Nce:     # Equality Constraints
%
% max_min: Vector (size=Nobj), indicating whether to minimize objective [1] 
%          or maximize objective [-1]
%
%%%%%%% Problem Variable Definitions %%%%%%%%
% vartype: [Type of variable, # feasible values] 
%          Type -- (0) Continuous, (1) Discrete, (2) Uniform Integer
%          if continuous/integer, # feas_values = 0, else # feas_values > 0 
% varlim:  Variable Limits (Lower Bound, Upper Bound); order of variable
%          limits should be the sam as in vartype
% dvvec:   List of feasible values of discrete variables in the same order 
%          as in vartype; e.g., for a problem with two discrete variables,
%          which can respectively take the sets of values, [1, 4, 5] and 
%          [10.5, 12.5, 15.5], dvvec = [1, 4, 5, 10.5, 12.5, 15.5]
%
%%%%%%% Algorithm Input Parameters %%%%%%%%
% Npop:          Particle Population size
% itermax:       Maximum number of iterations allowed
% Fcallmax:      Maximum number of function calls allowed
% iter_nochange: # consecutive iterations with no change in global best
% Tolf_term:     Termination tolerance for objective function
% Tolv_term:     Termination tolerance for design variables
% Tolcon_eq:     Tolerance for equality constraint
%               % ALL three must be met for termination
% Initpop_gen:   Method used for initial pop generation [1. Sobols, 2. LHS]
%
%%%%%%% Algorithm Control Parameters %%%%%%%%
% alpha:    inertia coefficient in PSO
% betal:    local exploitation coefficient in PSO
% betag:    global exploration coefficient in PSO
% gammac0:  diversity scaling coefficient for continuous variables
% gammad0:  diversity scaling coefficient for discrete variables
% gammamin: minimum scaling parameter for Gaussian diversity coeff function
% lambdah:  size fraction coeffcient for fractional hypercube
% diviter:  Iterations intervals for applying diversity preservation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%%%%%%% Algorithm Run Options %%%%%%%%
% display_iter: indicates whether to display best soln @ each iter (1: yes)
% plot_conv:    indicates whether to plot convergence real-time (1: yes)
% plot_interval:indicates interval at which convergence is plotted
% use_parallel: use parallel computing or not (1: yes)
% header:  string (<6 character) to head the saved population files 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% shuffling the random generators
%rng('default');
rng('shuffle'); % shuffling the random generators

exit_flag = 0;


%% Reading and Simplifying Problem Variable Properties
if length(func_param) ~= 7
    error('Too few or too many function inputs!');
end
Nv = func_param(1);
Nvc = func_param(2);
Nvd = func_param(3);
Nobj = func_param(4);
Nc = func_param(5);
Ncie = func_param(6);
Nce = func_param(7);

if Nvc+Nvd~=Nv || Ncie+Nce~=Nc
    error('Incorrect entries for function inputs');
end

if length(max_min) ~= Nobj
    error('size of max/min vector not the same as the number of objectives');
end

kdvec = 0;
for j = 1:Nv,
    vardef(j).lim = varlim(j,:);
    vardef(j).type = vartype(j,1);
    vardef(j).dveclen = vartype(j,2);
    vardef(j).dvec = [];
    if vartype(j,1) == 1, % discrete (not integer)
        vardef(j).dvec = dvvec(kdvec+j:kdvec+vartype(j,2));
        kdvec = kdvec + vartype(j,2);
    elseif vartype(j,1) == 2, % uniform integer
        vardef(j).dvec = varlim(j,1):1:varlim(j:2);
        vardef(j).dveclen = varlim(j,2)-varlim(j,1)+1;
    end
end


%% Generating Algorithm Input Parameters
if isempty(algo_inputs) == 1
    algo_inputs = MDPSO_input_param(func_param);
end
Npop = cell2mat(algo_inputs(1));
itermax = cell2mat(algo_inputs(2)); 
fcallmax = cell2mat(algo_inputs(3));
iter_nochange = cell2mat(algo_inputs(4));
Tolf_term = cell2mat(algo_inputs(5)); 
Tolv_term = cell2mat(algo_inputs(6)); 
Tolcon_eq = cell2mat(algo_inputs(7));
initpop_gen = cell2mat(algo_inputs(8));


%% Generating Algorithm Control Parameters
if isempty(algo_control) == 1
    algo_control = MDPSO_control_param(func_param);
end
alpha = cell2mat(algo_control(1));
betal = cell2mat(algo_control(2));
betag = cell2mat(algo_control(3));
gammac0 = cell2mat(algo_control(4));
gammad0 = cell2mat(algo_control(5));
gammamin = cell2mat(algo_control(6));
lambdah = cell2mat(algo_control(7));
diviter = cell2mat(algo_control(8));

% initializing diversity coefficients
gammac = 0;
gammad = zeros(1,Nv);


%% Generate Algorithm Run Options
if isempty(run_options) == 1
    run_options = MDPSO_run_options();
end
display_iter = cell2mat(run_options(1));
plot_conv = cell2mat(run_options(2));
plot_interval = cell2mat(run_options(3));
use_parallel = cell2mat(run_options(4));
header = run_options{5};

%% starting the pool of parallel workers
% if exist('parpool'),
%     if use_parallel ~= 1,
%         delete(gcp('nocreate'));
%     else
%         poolobj = gcp('nocreate'); % If no pool, do not create new one.
%         if isempty(poolobj),
%             parpool;
%         end
%     end
% end


%% Initializing the Population and Storing it to File
disp('Initializing the particle population.');

if initpop_gen == 1,
    P_sobol = sobolset(Nv); 
    varvec = net(P_sobol,Npop);
elseif initpop_gen == 2
    varvec = lhsdesign(Npop,Nv);
end

% scaling up variables to their actual values
particle = struct('var',[],'obj',[],'con',[],'con_eq',[],'con_ineq',[],'varvel',[]); % stores the variable, obj, constraint values, and velocities of particles
for i = 1:Npop,
    particle(i).var = varlim(:,1)' + varvec(i,:) .* (varlim(:,2) - varlim(:,1))';
    
    ub=min(norm(varargin{1})/cos(particle(i).var(1,2)),varargin{1,7});
    lb=norm(varargin{1});
%     particle(i).var(1,3)=(ub-lb)*particle(i).var(1,3) +lb;
    % initializing particle velocities to be zero
    particle(i).varvel = zeros(1,Nv);
end

% updating the discrete portion of the design vector
if Nvd > 0
    for ii=1:Npop,
        particle(ii).var = discrete_update(particle(ii).var,vardef, [], Nv, 0, 0); % gammad specified as [] to force deterministic update
    end
end

% evaluating the function value of solutions
funceval = str2func(func);
fcall = 0;
for ii=1:Npop,
    % calling the function file
    [fobj, fcon_eq, fcon_ineq] = funceval(particle(ii).var, varargin{:});
    fcall = fcall + 1;
    particle(ii).obj = fobj;
    particle(ii).con_eq = fcon_eq;
    particle(ii).con_ineq = fcon_ineq;
    % estimating the net constraint violation
    particle(ii).con = sum(max(fcon_ineq,0)) + sum(max(abs(fcon_eq)-Tolcon_eq,0));
end

% saving the initial population to a file
save([header,'_initial_population'], 'particle'); 


%% finding the initial global best solutions
globalbest = struct('var',[],'obj',[],'con',[],'con_eq',[],'con_ineq',[],'varvel',[]); % stores the best global particle
globalbestlog = struct('var',[],'obj',[],'con',[],'con_eq',[],'con_ineq',[],'varvel',[]); % stores best global particle in each iteration for writing to file
iter = 0;

% for single-objective problem
if Nobj == 1,
    igbest = 1;
    for ipop=2:Npop,
        sol1 = [particle(igbest).obj, particle(igbest).con];
        sol2 = [particle(ipop).obj, particle(ipop).con];
        if solution_comparison(sol1,sol2,Nobj,Nc,max_min) == 2 % identifying the better solution
            igbest = ipop; % stores the index of the best global particle
        end
    end
    globalbest = particle(igbest);
end
clear sol1 sol2

globalbestlog(iter+1) = globalbest;
% save('initial_best','globalbest');
save([header,'_convhist'],'globalbestlog')
if display_iter == 1,
    disp('The best particle or solution at each iteration:');
    fprintf('Iteration\t Func_Call\t Best_Particle\t Objective \t Constraint\n');
    fprintf('%9.0f\t %9.0f\t %13.0f\t %9.5g\t %10.5g\n', iter, fcall, igbest, globalbest.obj, globalbest.con);
end

if plot_conv == 1
    fig_convhist = figure('name','fig_convhist');
    subplot(1,2,1), H1 = plot(iter,globalbest.obj);
    title('Convergence History (Objective)');
    set(H1,'Marker','o','MarkerSize',10');
    xlabel('# Iterations');
    ylabel('Objective Function');
    hold on
    subplot(1,2,2), H2 = plot(iter,globalbest.con);
    title('Convergence History (Net Constraint Violation)');
    set(H2,'Marker','x','MarkerSize',10');
    xlabel('# Iterations');
    ylabel('Net Constraint Violation')
    hold on;
end


%% initializing the local best solutions
localbest = struct('var',[],'obj',[],'con',[],'con_eq',[],'con_ineq',[],'varvel',[]);
for ipop=1:Npop,
    localbest(ipop) = particle(ipop);
end


%% Starting MDPSO Iterations
terminate_flag = iter_nochange; % when terminate flag becomes zero, iterations are terminated
pvar_mat = zeros(Npop,Nv); % stores the design variable vector for all particles (to send to diversity evaluation)
idiv = 1; % counting when diversity preservation was applied in order to log diversity parameters

while iter<itermax && fcall<=fcallmax && terminate_flag>0
    iter = iter + 1;

    %% Estimating the diversity coefficient
    % apply diversity preservation only at diviter intervals and if not
    % suppressed by gammac0,gammad0 = 0
    if rem(iter, diviter) == 0 && (gammac0 + gammad0) > 0
        for ipop=1:Npop,
            pvar_mat(ipop,:) = particle(ipop).var;
        end
        [gammac, gammad, D_continuous, D_discrete] = diversity_eval(pvar_mat,globalbest,vardef,gammac0,gammad0,gammamin,lambdah);
        
        % log the diversity parameter values
        gammaclog(idiv) = gammac; gammadlog(idiv,:) = gammad; 
        D_continuouslog(idiv) = D_continuous; D_discretelog(idiv,:) = D_discrete;
        idiv = idiv + 1;
    end
    
    
    %% Updating the particle locations
    for ipop=1:Npop,
        %% updating all variables based on continuous PSO dynamics
        % generating the random parameters (local, global, diversity_cont)
        r_l = rand;
        r_g = rand;
        r_dc = rand;
        
        % updating particle velocities
        vel_intertia = alpha * particle(ipop).varvel; % intertia term
        vel_local = betal * r_l * (localbest(ipop).var - particle(ipop).var); % exploitive/local term
        vel_global = betag * r_g * (globalbest.var - particle(ipop).var); % explorative/global term
        
        % adding diversity term to particle velocity
        vel_div = zeros(1,Nv); % diversity term
        if rem(iter, diviter) == 0 && gammac ~= 0,
            vel_div = - gammac * r_dc * (globalbest.var - particle(ipop).var); % diversity term
        end
        
        pvarvel = vel_intertia + vel_local + vel_global + vel_div;
        
        % adjusting particle location to prevent going out of bounds
        pvar = particle(ipop).var + pvarvel;
        for j=1:Nv,
            if pvarvel(j) > 0 && pvar(j) > vardef(j).lim(2) % checking upper bound
                pvarvel(j) = vardef(j).lim(2) - particle(ipop).var(j);
            elseif pvarvel(j) < 0 && pvar(j) < vardef(j).lim(1) % checking lower bound
                pvarvel(j) = vardef(j).lim(1) - particle(ipop).var(j);
            end
        end        
        % updating particle location
        pvar =  particle(ipop).var + pvarvel; % storing new particle location in a temporary vector
        
        %% applying the discrete variable update (nearest vertex principle)
        if Nvd > 0
            pvar = discrete_update(pvar, vardef, gammad, Nv, iter, diviter);
            for j=1:Nv,
                if vardef(j).type ~= 0 % for discrete variables
                    pvarvel(j) = pvar(j) - particle(ipop).var(j);
                end
            end
        end
        
        particle(ipop).varvel = pvarvel;
        particle(ipop).var = pvar;
        
        
        %% estimating the objective functions and contraints for new particle location
        % calling the function file
        [fobj, fcon_eq, fcon_ineq] = funceval(particle(ipop).var, varargin{:});
        fcall = fcall + 1;
        particle(ipop).obj = fobj;
        particle(ipop).con_eq = fcon_eq;
        particle(ipop).con_ineq = fcon_ineq;
        % estimating the net constraint violation
        particle(ipop).con = sum(max(fcon_ineq,0)) + sum(max(abs(fcon_eq)-Tolcon_eq,0));
        
        
        %% updating the local best (historical best) for each particle
        sol1 = [localbest(ipop).obj, localbest(ipop).con];
        sol2 = [particle(ipop).obj, particle(ipop).con];
        if solution_comparison(sol1,sol2,Nobj,Nc,max_min) == 2 % identifying the beter solution
            localbest(ipop) = particle(ipop); % updating localbest if current particle is better
        end
    end
    clear sol1 sol2
        
    
    %% updating the global best particle
    % for single-objective problem
    if Nobj == 1,
        flag_change_global = 0;
        sol1 = [globalbest.obj, globalbest.con];
        for ipop = 1:Npop,
            sol2 = [localbest(ipop).obj, localbest(ipop).con]; % comparing with localbest solutions and not with particles
            if solution_comparison(sol1,sol2,Nobj,Nc,max_min) == 2 % identifying the better solution
                igbest = ipop; % gives the particle, whose hostorical best will become the global best
                sol1 = [localbest(ipop).obj, localbest(ipop).con];
                flag_change_global = 1;
            end
        end
        if flag_change_global == 1
            globalbest = localbest(igbest); % assigning the best localbest solution to become the global best
        end
    end
    clear sol1 sol2
    
    
    %% saving the current globalbest to file, and displaying it on command window and convergence plot
    globalbestlog(iter+1) = globalbest;
    save([header,'_convhist'],'globalbestlog')
    if display_iter == 1,
        fprintf('%9.0f\t %9.0f\t %13.0f\t %9.5g\t %10.5g\n', iter, fcall, igbest, globalbest.obj, globalbest.con);
    end

    if plot_conv == 1 && rem(iter, plot_interval) == 0,
        figure(fig_convhist);        
        subplot(1,2,1), H1 = plot(iter,globalbest.obj);
        set(H1,'Marker','o','MarkerSize',10');
        hold on
        subplot(1,2,2), H2 = plot(iter,globalbest.con);
        set(H2,'Marker','x','MarkerSize',10');
        hold on;
    end
 
    
    %% estimating the termination criteria
    if Nobj == 1,
        obj_change = abs(globalbest.obj - globalbestlog(iter).obj);
        con_change = abs(globalbest.con - globalbestlog(iter).con);
        var_change = norm(globalbest.var - globalbestlog(iter).var);
        
        if obj_change < Tolf_term && con_change < Tolf_term && var_change < Tolv_term
            terminate_flag = terminate_flag - 1; % marks another consecutive iteratio with "less than tolerance" change in obj, con, and var
        else
            terminate_flag = iter_nochange;
        end
    end
end
hold off

if iter>=itermax,
    fprintf('Iterations terminated. Maximum allowed number of iterations exceeded.\n\n');
    exit_flag = 1;
elseif fcall > fcallmax
    fprintf('Iterations terminated. Maximum allowed number of function calls exceeded.\n\n');
    exit_flag = 2;
elseif terminate_flag == 0,
    fprintf('Iterations terminated. Changes in function and variable values of the global best solution were less than tolerance values in %d consecutive iterations.\n\n', iter_nochange);
    exit_flag = 3;
end

% saving the diversity log
save([header,'_divhist'],'gammaclog', 'gammadlog', 'D_continuouslog', 'D_discretelog');

% saving the final population
save([header,'_final_population'], 'particle'); 


%% plotting the convergence history, reporting the optimum, and animating 2D optimization
% displaying optimum solution
fprintf('Optimum objective function:  %10.3e\n',globalbest.obj);
fprintf('Net constraint violation:    %10.3e\n',globalbest.con);
fprintf('Optimum design vector:   \n');
disp((globalbest.var)');

% plotting convhist if not plotted during iteration
bestfunc = zeros(iter+1,Nobj+1);
if plot_conv ~= 1
    figure(1);
    for it = 1:iter+1,
        bestfunc(it,:) = [globalbestlog(it).obj, globalbestlog(it).con];
    end
    %figure('name','fig_convhist');
    subplot(1,2,1), H1 = plot((1:iter+1),bestfunc(:,1:Nobj));
    title('Convergence History (Objective)');
    set(H1,'Marker','o','MarkerSize',10','MarkerEdgeColor','b');
    xlabel('# Iterations');
    ylabel('Objective Function');
    subplot(1,2,2), H2 = plot((1:iter+1),bestfunc(:,Nobj+1));
    title('Convergence History (Net Constraint Violation)');
    set(H2,'Marker','x','MarkerSize',10','MarkerEdgeColor','b');
    xlabel('# Iterations');
    ylabel('Net Constraint Violation')
end

% animating the population movement if there are two variables
% for it = 1:iter+1,
%     bestfunc(it,:) = [globalbestlog(it).obj, globalbestlog(it).con];
% end
%%

% matlabpool('close');












