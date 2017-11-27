clear all
close all
clc
% data for training
sl1=0;
sl2=0;
tot=0;
for i =1:4
    ang=i*90;
    flname=sprintf('processed_sc_dc_%d.mat',ang);
    load(flname);
    
    for j=1:length(dc_feasible)
      if min_sep_dc(j,1)<10 && min_sep_dc(j,1)>1.5
          sl1=sl1+1;
      theta_dc(sl1,1)=atan2(dc_feasible(j).Vb(2,1),dc_feasible(j).Vb(1,1))-atan2(dc_feasible(j).Va(2,1),dc_feasible(j).Va(1,1));
      rel_speed_dc(sl1,:)=(dc_feasible(j).Vb)'-(dc_feasible(j).Va)';
      rel_dist_dc(sl1,:)=(dc_feasible(j).Pb)';
      energy_dc(sl1,1)=dc_feasible(j).fx2;
      x1_dc(sl1,1)=dc_feasible(j).X2(1,1);
      x2_dc(sl1,1)=dc_feasible(j).X2(1,2);
      sep_dc(sl1,1)=min_sep_dc(j,1);
      end
   
    end

for j=1:length(sc_feasible)
    if min_sep_sc(j,1)<10 && min_sep_sc(j,1)>1.5
          sl2=sl2+1;
      theta_sc(sl2,1)=atan2(sc_feasible(j).Vb(2,1),sc_feasible(j).Vb(1,1))-atan2(sc_feasible(j).Va(2,1),sc_feasible(j).Va(1,1));
      rel_speed_sc(sl2,:)=(sc_feasible(j).Vb)'-(sc_feasible(j).Va)';
      rel_dist_sc(sl2,:)=(sc_feasible(j).Pb)';
      energy_sc(sl2,1)=sc_feasible(j).fx2;
      x1_sc(sl2,1)=sc_feasible(j).X1(1,1);
      x2_sc(sl2,1)=sc_feasible(j).X1(1,2);
      sep_sc(sl2,1)=min_sep_sc(j,1);
    end
   
end
end
 inp_dc=[rel_speed_dc rel_dist_dc];
 inp_sc=[rel_speed_sc rel_dist_sc];