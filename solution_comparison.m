% This program performs the comparison of two solutions (constrained,
% unconstrained, single=- and multi-objective)
% It uses the principle of contrained non-dominance

function better_solution = solution_comparison(solution1,solution2,Nobj,Nc,max_min)

%%%%% Defining the Function Inputs and Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solution1, solution2: vectors with values of Nobj objectives functions,
%               with the last element giving the net constraint violation
% better_solution: indicates the dominating solution (1 or 2), 
%                  returns 1 or 2 randomly, if both solutions are equal
%                  returns 0, if solutions are non-dominated
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% coverting each objective into a minimization objective (maximization
% objectives are multiplied by -1)
solution1(1:Nobj) = max_min.*solution1(1:Nobj);
solution2(1:Nobj) = max_min.*solution2(1:Nobj);

%% comparing solutions
if Nc > 0 && solution1(Nobj+1)~=solution2(Nobj+1), % constrained or unequal constraint violation
    if solution1(Nobj+1) < solution2(Nobj+1)
        better_solution = 1;
    else
        better_solution = 2;
    end
else % unconstrained or equal constraint violation
    if Nobj == 1 && solution1(1) == solution2(1)% for single-obj problems, choose randomly
        better_solution = 1 + round(rand(1,1));
    elseif solution1(1:Nobj) <= solution2(1:Nobj) % sol 1 is better
        better_solution = 1;
    elseif solution2(1:Nobj) <= solution1(1:Nobj) % sol 2 is better
        better_solution = 2;
    else
        better_solution = 0; % non-dominated solutions for multi-obj prob 
    end
end

end