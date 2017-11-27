%% This function file generates the input parameters for the MDPSO 
%% algorithm

function algo_inputs = MDPSO_input_param(varargin)


%%%%% Defining MDPSO_input_param %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% algo_inputs = MDPSO_input_param(func_param,'Popsize',value,'Itermax',value,'Fcallmax',value, ... 
% 'IterNoChange',value,'Tolfterm',value,'Tolvterm',value,'Tolconeq',value,'Initpopgen',value)
%
% value for 'Initpop' can be 'Sobols' or 'LHS'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% entering defualt values (all tolerances are relative tolerances)
func_param = varargin{1};
Nv = func_param(1);
Npop = 10 * Nv;  % following the concept of small set
itermax = 50;
fcallmax = 0; 
iter_nochange = 5;
Tolf_term = 1e-09; 
Tolv_term = 1e-09; 
Tolcon_eq = 1e-06;
initpop_gen = 1;


%% Checking User Inputs to Overwrite the Default Values
if nargin>1,
    for i=2:2:nargin,
        param = varargin{i};
        value = varargin{i+1};
        switch param
            case 'Popsize'
                Npop = value;
                continue;
            case 'Itermax'
                itermax = value;
                continue;
            case 'Fcallmax'
                fcallmax = value;
            case 'IterNoChange'
                iter_nochange = value;
                continue;
            case 'Tolfterm'
                Tolf_term = value;
                continue;
            case 'Tolvterm'
                Tolv_term = value;
                continue;
            case 'Tolconeq'
                Tolcon_eq = value;
                continue;
            case 'InitpopGen'
                if strcmp(value,'Sobols') == 1
                    initpop_gen = 1;
                elseif strcmp(value,'LHS') == 1
                    initpop_gen = 2;
                else
                    error('Wrong name for population initialization method!');
                end
            otherwise
                error('The algorithm input type, "%s", does not exist!',param)
        end
    end
end

% updating dependent algo parameters
if fcallmax == 0
    fcallmax = Npop * itermax; 
end


%% Saving Return Value (in Cell Format)
algo_inputs = {Npop, itermax, fcallmax, iter_nochange, Tolf_term, ... 
               Tolv_term, Tolcon_eq, initpop_gen};

end


