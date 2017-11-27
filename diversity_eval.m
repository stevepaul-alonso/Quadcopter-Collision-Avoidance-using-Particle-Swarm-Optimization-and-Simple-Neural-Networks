% This function evaluates the diversity metric and the diversity
% coefficients

function [gammac, gammad, D_continuous, D_discrete] = ...
diversity_eval(pvar_mat,globalbest,vardef,gammac0,gammad0,gammamin,lambdah)

[Npop, Nv] = size(pvar_mat);
varbest = globalbest.var;

% initializing the diversity coefficients
gammad = zeros(1,Nv);
D_discrete = zeros(1,Nv);

%% estimating the smallest and lambda fractional hypercube enclosing the population
hyper_continuous = 1; % stores the hypercube value for continuous variable
varU = zeros(1,Nv);
varL = zeros(1,Nv);
varR = zeros(1,Nv);
for jj = 1:Nv,
    varU(jj) = max(pvar_mat(:,jj)); % variable upper bound of current poulation
    varL(jj) = min(pvar_mat(:,jj)); % variable lower bound of current poulation
    varR(jj) = varU(jj) - varL(jj); % variable range bound of current poulation
    
    % estimating smallest enclosing hypercube
    hyper_continuous = hyper_continuous * varR(jj) / (vardef(jj).lim(2) - vardef(jj).lim(1));
    
    % estimating gammad to implement stochastic update of the discrete variables
    if vardef(jj).type ~= 0
        D_discrete(jj) = varR(jj) / (vardef(jj).lim(2) - vardef(jj).lim(1)); 
    end
    
    % estimating the lambda fractional hypercube bounds
    varU(jj) = max((varL(jj) + lambdah*varR(jj)), min(varbest(jj) + 0.5*lambdah*varR(jj), varU(jj))); % adjusting the upper bound of lambda fractional domain
    varL(jj) = min((varU(jj) - lambdah*varR(jj)), max(varbest(jj) - 0.5*lambdah*varR(jj), varL(jj))); % adjusting the lower bound of lambda fractional domain    
end


%% estimating number of particles in the lambda fractional domain
Npopfrac = 0;
for ii = Npop,
    if pvar_mat(ii,:) < varU
        if pvar_mat(ii,:) > varL,
            Npopfrac = Npopfrac + 1;
        end
    end
end


%% adjusting the fractional hypercube and calculating the diversity metrics and coefficients
div_adjust = (lambdah * (Npop + 1)/(Npopfrac + 1))^(1/Nv); % to adjust for outliers

D_continuous =  div_adjust * hyper_continuous^(1/(Nv));
sigma_continuous = 1 / sqrt(2 * log(1/gammamin));
gammac = gammac0 * exp(-D_continuous^2 / (2*sigma_continuous^2));


for jj = 1:Nv,
    if vardef(jj).type ~= 0,
        D_discrete(jj) = div_adjust * D_discrete(jj);
        sigma_discrete = 1 / sqrt(2*log(vardef(jj).dveclen));
        gammad(jj) = gammad0 * exp(-D_discrete(jj)^2/(2*sigma_discrete^2)); 
    end
end

end