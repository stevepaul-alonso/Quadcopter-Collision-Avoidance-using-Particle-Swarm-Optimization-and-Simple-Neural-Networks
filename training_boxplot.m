clear all
close all
clc

load('training_boxplot2.mat')
count_dc=[0;0;0;0];
for i =1:length(approach_angle)
    if approach_angle(i,1)<45
        count_dc(1,1)=count_dc(1,1)+1;
        En_dc_45(count_dc(1,1),1)= Inc_En_dc(i,1)*100;
        En_sc_45(count_dc(1,1),1)= Inc_En_sc(i,1)*100;
        Sep_dc_45(count_dc(1,1),1) = sep(i,1);
        Sep_sc_45(count_dc(1,1),1) = sep(i,2);
    elseif approach_angle(i,1)>=45 && approach_angle(i,1)<90
         count_dc(2,1)=count_dc(2,1)+1;
        En_dc_90(count_dc(2,1),1)= Inc_En_dc(i,1)*100;
        En_sc_90(count_dc(2,1),1)= Inc_En_sc(i,1)*100;
        Sep_dc_90(count_dc(2,1),1) = sep(i,1);
        Sep_sc_90(count_dc(2,1),1) = sep(i,2);
    elseif approach_angle(i,1)>=90 && approach_angle(i,1)<135
         count_dc(3,1)=count_dc(3,1)+1;
        En_dc_135(count_dc(3,1),1)=Inc_En_dc(i,1)*100;
        En_sc_135(count_dc(3,1),1)= Inc_En_sc(i,1)*100;
        Sep_dc_135(count_dc(3,1),1) = sep(i,1);
        Sep_sc_135(count_dc(3,1),1) = sep(i,2);
     elseif approach_angle(i,1)>=135 && approach_angle(i,1)<=180 
          count_dc(4,1)=count_dc(4,1)+1;
        En_dc_180(count_dc(4,1),1)= Inc_En_dc(i,1)*100;
        En_sc_180(count_dc(4,1),1)= Inc_En_sc(i,1)*100;
        Sep_dc_180(count_dc(4,1),1) = sep(i,1);
        Sep_sc_180(count_dc(4,1),1) = sep(i,2);
    end
    
end
p_45 = zeros(length(En_dc_45),1);
p_90 = ones(length(En_dc_90),1);
p_135 = ones(length(En_dc_135),1) + 1;
p_180 = ones(length(En_dc_180),1)+2;
boxplot([En_dc_45' En_dc_90' En_dc_135' En_dc_180'],[p_45' p_90' p_135' p_180'],'Labels',{'0 - 45','45 - 90','90 - 135','135 - 180'})
title('Increase in energy consumption for direction change')
xlabel('Angle (Degree)')
ylabel('% increase in energy consumption')
figure
boxplot([En_sc_45' En_sc_90' En_sc_135' En_sc_180'],[p_45' p_90' p_135' p_180'],'Labels',{'0 - 45','45 - 90','90 - 135','135 - 180'})
title('Increase in energy consumption for speed change')
xlabel('Angle (Degree)')
ylabel('% increase in energy consumption')
figure
boxplot([Sep_dc_45' Sep_dc_90' Sep_dc_135' Sep_dc_180'],[p_45' p_90' p_135' p_180'],'Labels',{'0 - 45','45 - 90','90 - 135','135 - 180'})
hold on
plot([-0.5 4.5],[1.5 1.5])
title('Minimum separation distance for direction change')
xlabel('Angle (Degree)')
ylabel('Minimum separation (m)')
figure
boxplot([Sep_sc_45' Sep_sc_90' Sep_sc_135' Sep_sc_180'],[p_45' p_90' p_135' p_180'],'Labels',{'0 - 45','45 - 90','90 - 135','135 - 180'})
hold on
plot([-0.5 4.5],[1.5 1.5])
title('Minimum separation distance for speed change')
xlabel('Angle (Degree)')
ylabel('Minimum separation (m)')