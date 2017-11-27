%% This function generates the run options for MDPSO

function run_options = MDPSO_run_options(varargin)


%%%%% Defining MDPSO_run_options %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run_options = MDPSO_run_options('display', value ,plotconv, value, 'plotinterval', value)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Generting Default Values of Run Options
display_iter = 1;
plot_conv = 1;
plot_interval = 5;
use_parallel = 1;
header = 'mdpso';


%% Checking User Inputs to Overwrite the Default Values
if nargin>1,
    for i=1:2:nargin,
        param = varargin{i};
        value = varargin{i+1};
        switch param
            case 'display'
                if strcmp(value,'on')==1 || strcmp(value,'ON')==1
                    display_iter = 1;
                else
                    display_iter = 0;
                end
                continue;
            case 'plotconv'
                if strcmp(value,'on')==1 || strcmp(value,'ON')==1
                    plot_conv = 1;
                else
                    plot_conv = 0;
                end
                continue;
            case 'plotinterval'
                plot_interval = value;
                continue;
            case 'useparallel'
                if strcmp(value,'on')==1 || strcmp(value,'ON')==1
                    use_parallel = 1;
                else
                    use_parallel = 0;
                end
                continue;
            case 'header'
                header = value;
                if length(header)>5
                    header = header(1:5);
                end
                continue;
            otherwise
                error('This run option type does not exist!')
        end
    end
end


%% Saving Return Value (in Cell Format)
run_options = {display_iter, plot_conv, plot_interval, use_parallel, header};

end

