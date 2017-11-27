% This function performs the discrete variable update based on the concept
% of nearest hypercube vertex, coupled with a stochastic update to
% preserve diversity (prevent getting stuck in discrete cells)
%
% Modified by Ben Rinauto.
% - Boundary Checking added
function pvar_update = discrete_update(pvar, vardef, gammad, Nv, iter, diviter)

pvar_update = pvar;

if isempty(gammad)
    gammad = -ones(1,Nv);
end

%% updating the discrete portion of the design variable vector
for jj=1:Nv,
    r_dd = rand(1,1); % random parameter deciding if discrete variable diversity preservation is to be applied
    r_choice = rand(1,1); % random updator, in case diversity is applied (stochastic update)
    
    % if the variable is continuous
    if vardef(jj).type == 1, 
        continue;
     
    % if the variable is an uniform integer    
    elseif vardef(jj).type == 2, 
        if rem(iter, diviter) == 0 && r_dd < gammad(jj) % stochastic update - diversity preservation
            pvar_update(jj) = round(floor(pvar(jj)) + r_choice);
        else
            pvar_update(jj) = round(pvar(jj)); % deterministic update - immediate nearest integer
        end
        
        % Check to make sure var did not go out of bounds.  If it did, move
        % it to the boundary.
        if(pvar_update(jj) < vardef(jj).lim(1))
            pvar_update(jj) = vardef(jj).lim(1);
        elseif(pvar_update(jj) > vardef(jj).lim(2))
            pvar_update(jj) = vardef(jj).lim(2);
        end
        
        
    % if the variable is a non uniform discrete variable    
    elseif vardef(jj).type == 1, 
        [dvec_diff, dvec_index] = min(abs(vardef(jj).dvec - pvar(jj))); % identifying the nearest feasible value (NFV)
        
        if rem(iter, diviter) == 0 && r_dd < gammad(jj) % stochastic update - diversity preservation
            index_move = sign(pvar(jj) - vardef(jj).dvec(dvec_index)); % figuring out if the NFV is greater/lower than the variable value
            dvec_index = round(dvec_index + (index_move*r_choice)); % randomly selecting one of the two immediate discrete values
            pvar_update(jj) = vardef(jj).dvec(dvec_index); 
        else
            pvar_update(jj) = vardef(jj).dvec(dvec_index); % deterministic update to NFV
        end
        
    end
end

end