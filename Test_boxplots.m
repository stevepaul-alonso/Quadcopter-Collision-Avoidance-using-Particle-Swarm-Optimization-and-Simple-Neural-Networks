%% testing boxplot
clear all
close all
clc
load('Test_boxplots.mat')

En_Inc = min((Energy_col - Energy_no_col)./Energy_no_col.*100,100);
boxplot(En_Inc,'Labels',{''})
ylabel('% increase in energy consumption')
title('Increase in energy consumption for collisin avoidance')
figure
boxplot(minsep(1:500,1),'Labels',{''})
ylabel('minimum separation distance (m)')
title('Minimum separation distance acheived')
hold on
plot([.5 1.5],[1.5 1.5])