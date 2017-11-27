clear all
close all
clc

%% process 1

% load('results_set1.mat')
% 
% V_max=15;
% t_trans=7;
% UAV_size=.5;
% safety_factor=3;
% d_thresh=UAV_size*safety_factor;
% n_max=500;
% map_x=500*UAV_size;
% t_trans=10;
% 
% for i=1: length(combined)
%     Pa_i=combined(i).Pa;
%     Pb_i=combined(i).Pb;
%     V1=combined(i).Va;
%     V2=combined(i).Vb;
%     Pa_f=Pa_i + V1.*t_trans;
%     Pb_f=Pb_i + V2.*t_trans;
%     [detect tcol]=col_det2(Pa_i,V1,Pb_i,V2);
%     t_15=time_to_min_dist_15(Pa_i,V1,Pb_i,V2);
%     tc1=transition_begin_time(V1,V2,Pa_i,Pb_i);
%         tc=tc1;
%     X1=combined(i).X1;
%     X2=combined(i).X2;
%    [total_energy ,C_eq, C_ineq]= visual_sim(X1,V1, V2, tc, Pa_i, Pb_i, Pa_f, Pb_f, V_max,tc1,t_trans,2,tcol);
%    min_sep(i,1)=3-C_ineq(2);
%    [total_energy ,C_eq, C_ineq]= visual_sim(X2,V1, V2, tc, Pa_i, Pb_i, Pa_f, Pb_f, V_max,tc1,t_trans,1,tcol); 
%    min_sep(i,2)=3-C_ineq(2);
% end
% SAVED the results of part 1 as reult_processed1.mat
%% Process 2
load('result_processed1.mat')
for i=1:length(combined)
%     theta1=rad2deg(atan2(combined(i).Va(2,1),combined(i).Va(1,1)));
%     theta2=rad2deg(atan2(combined(i).Vb(2,1),combined(i).Vb(1,1)));
      delta_v=combined(i).Va-combined(i).Vb;    
theta(i,1)=abs(rad2deg(atan2(delta_v(2,1),delta_v(1,1))));
end
for i=1:length(min_sep)
    if min_sep(i,1)==6, min_sep(i,1)=0; end
    if min_sep(i,2)==6, min_sep(i,2)=0; end
    if min_sep(i,1)>1.5, min_sep(i,3)=1; else min_sep(i,3)=0; end
    if min_sep(i,2)>1.5, min_sep(i,4)=1; else min_sep(i,4)=0; end
end
t_45=[0,0,0]; t_90=[0,0,0]; t_135=[0,0,0]; t_180=[0,0,0];
for i=1:length(theta)
    if theta(i,1)<=45 && min_sep(i,3)==1
        t_45(1,1)=t_45(1,1)+1;
    end
    if theta(i,1)<=90 && theta(i,1)>45 && min_sep(i,3)==1
        t_90(1,1)=t_90(1,1)+1;
    end
    if theta(i,1)<=135 && theta(i,1)>90 && min_sep(i,3)==1
        t_135(1,1)=t_135(1,1)+1;
    end
    if theta(i,1)<=180 && theta(i,1)>135 && min_sep(i,3)==1
        t_180(1,1)=t_180(1,1)+1;
    end
    
    if theta(i,1)<=45 && min_sep(i,4)==1
        t_45(1,2)=t_45(1,2)+1;
    end
    if theta(i,1)<=90 && theta(i,1)>45 && min_sep(i,4)==1
        t_90(1,2)=t_90(1,2)+1;
    end
    if theta(i,1)<=135 && theta(i,1)>90 && min_sep(i,4)==1
        t_135(1,2)=t_135(1,2)+1;
    end
    if theta(i,1)<=180 && theta(i,1)>135 && min_sep(i,4)==1
        t_180(1,2)=t_180(1,2)+1;
    end
        
end
for i=1:length(theta)
    if theta(i,1)<=45 
        t_45(1,3)=t_45(1,3)+1;
    end
    if theta(i,1)<=90 && theta(i,1)>45 
        t_90(1,3)=t_90(1,3)+1;
    end
    if theta(i,1)<=135 && theta(i,1)>90 
        t_135(1,3)=t_135(1,3)+1;
    end
    if theta(i,1)<=180 && theta(i,1)>135 
        t_180(1,3)=t_180(1,3)+1;
    end
end
        
bar([t_45;t_90;t_135;t_180])
ylabel('numbers')
xlabel('angle in degrees')
legend('DC','SC','Total')
