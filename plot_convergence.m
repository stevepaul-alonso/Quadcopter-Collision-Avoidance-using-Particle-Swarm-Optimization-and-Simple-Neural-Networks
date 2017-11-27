% This function plots the convergence history of MDPSO

function plot_convergence()

if plot_conv == 1
    figure(1);
    [AX,H1,H2] = plotyy(iter,globalbest.obj,iter,globalbest.con);
    title('MDPSO Convergence History');
    xlabel('# Iterations');
    ylabel(AX(1),'Objective Function')
    ylabel(AX(2),'Net Constraint Violation')
    legend(AX(1),'Objective Function','Location','northwest');
    legend(AX(2),'Net Constraint Violation','Location','northeast');
    set(H1,'Marker','o','MarkerSize',10');
    set(H2,'Marker','x','MarkerSize',10');
end