%% This function file generates the control parameters for the MDPSO
%% algorithm

function algo_control = MDPSO_control_param(varargin)


%%%%% Defining MDPSO_control_param %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% algo_control = MDPSO_control_param(func_param,'alpha',value,'betal',value,'betag',value, ... 
% 'gammac0',value,'gammad0',value,'gammamin',value,'lambdah',value,'diviter',value)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% entering default values (based on Chowdhury et al. SMO 2013)
func_param = varargin{1};
Nc = func_param(5);
Nvd = func_param(3);

alpha = 0.5;
betal = 1.4;
betag = 1.4;

% greater degree of diversity preservation for constrained problems
if Nc == 0
    gammac0 = 0.5;
    gammamin = 1.0e-10;
    lambdah = 0.25;
else
    gammac0 = 2.0;
    gammamin = 1.0e-05;
    lambdah = 0.1;
end
if Nvd == 0,
    gammad0 = 0.0;
else
    gammad0 = 0.5;
end

diviter = 1;


%% Checking User Inputs to Overwrite the Default Values
if nargin>1,
    for i=2:2:nargin,
        param = varargin{i};
        value = varargin{i+1};
        switch param
            case 'alpha'
                alpha = value;
                continue;
            case 'betal'
                betal = value;
                continue;
            case 'betag'
                betag = value;
                continue;
            case 'gammac0'
                gammac0 = value;
                continue;
            case 'gammad0'
                gammad0 = value;
                continue;
            case 'gammamin'
                gammamin = value;
                continue;
            case 'lambdah'
                lambdah = value;
                continue;
            case 'diviter'
                diviter = value;
                continue;
            otherwise
                error('This algorithm control parameter type does not exist!')
        end
    end
end


%% Saving Return Value (in Cell Format)
algo_control = {alpha, betal, betag, gammac0, gammad0, gammamin, ...
                lambdah, diviter};


end

