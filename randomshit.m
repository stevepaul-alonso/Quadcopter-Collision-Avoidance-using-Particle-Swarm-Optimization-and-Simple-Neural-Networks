clear all
close all
clc
load('opti_1.mat')
sl1=0;
angs=[0,45,90,135,180]';
counts=[0;0;0;0];
e_sc=[0;0;0;0];
e_dc=[0;0;0;0];
for i=1:length(dc_samples)
    sl1=sl1+1;
theta_dc(sl1,1)=acosd((dc_samples(i).Vb'*dc_samples(i).Va)/(norm(dc_samples(i).Vb)*norm(dc_samples(i).Va)));
delta_energy_dc(sl1,1)=(Energy(sl1,1)-Energy(sl1,3))/Energy(sl1,3);
delta_energy_sc(sl1,1)=(Energy(sl1,2)-Energy(sl1,4))/Energy(sl1,4);
for j=1:4
        if theta_dc(sl1,1)>angs(j,1) && theta_dc(sl1,1)<=angs(j+1,1)
            counts(j,1)=counts(j,1)+1;
            e_sc(j,1)=e_sc(j,1)+delta_energy_sc(sl1,1);
            e_dc(j,1)=e_dc(j,1)+delta_energy_dc(sl1,1);
            
        end
end
end
e_sc=e_sc./counts;
e_dc=e_dc./counts;