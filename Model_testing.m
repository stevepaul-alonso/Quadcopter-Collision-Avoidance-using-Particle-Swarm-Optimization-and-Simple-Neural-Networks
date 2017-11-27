clear all 
close all
clc

%% specify the assumptions
V_max=15; % assumed maximum speed for a UAV
t_det=5; % collision detection time
UAV_size=.5; % radius of UAV
safety_factor=3;
d_thresh=UAV_size*safety_factor; % collision defining distance
n_max=500; % number of samples
map_x=200*UAV_size; % map size
sl=0;
dc=0;
 sc=0;
 col=0;

points=create_sample_points_new(V_max,t_det,d_thresh,n_max,map_x);

for i=1:length(points)
     Pa_i=points(i).Pa;
    Pb_i=points(i).Pb;
    V1=points(i).Va;
    V2=points(i).Vb;
    org_sep(i,1) = simu(Pa_i,V1,Pb_i,V2);
    Pa_f=Pa_i + V1.*t_det;
    Pb_f=Pb_i + V2.*t_det;
    [detect tcol]=col_det2(Pa_i,V1,Pb_i,V2);
    t_15(i,1)=time_to_min_dist_15(Pa_i,V1,Pb_i,V2);
    Pb_i=Pb_i-V2.*(t_det-t_15(i,1));
    
%     load('separation.mat')
tic
    if detect==1
        
        sl=sl+1;
        
        rel_speed=(V2)'-(V1)';
        rel_dist=(Pb_i)';
%         theta=atan2(V2(2,1),V2(1,1))-atan2(V1(2,1),V1(1,1));
        theta = acosd((V1'*V2)/(norm(V1)*norm(V2)));
        theta2=atan2(V1(2,1),V1(1,1))-atan2(V2(2,1),V2(1,1));
        inp=[rel_speed rel_dist];
        inp_sc_A=[Pb_i' theta norm(V1) norm(V2)];
        inp_sc_B=[-Pb_i' theta norm(V2) norm(V1)];
        inp2=[-rel_speed -rel_dist];

        

    
          dist=norm(Pb_i);
          v1=norm(V1);
          v2=norm(V2);
%           approach_angle=rem((atan2(points(i).Vb(2,1),points(i).Vb(1,1))-atan2(points(i).Va(2,1),points(i).Va(1,1)))*180/pi + 360,180);
            approach_angle(i,1)=acosd(points(i).Vb'*points(i).Va/(norm(points(i).Va)*norm(points(i).Vb)));

         
          in=[approach_angle(i,1) v1 v2];


          meth=method_prediction(in);

          if meth >.5
               

           X1=X_dc_trained(inp);

           X2=X_dc_trained(inp);

        [total_energy]= original_motion(points(i),10,X1(1,1),X2(1,1));
        Energy_no_col(i,1) = total_energy;
        time_rec(sl,1)=toc;
           method(sl,1)=1;
          [total_energy ,C_eq, C_ineq]= visual_sim_2_agent_test(X1,X2,V1, V2, Pa_i, Pb_i, Pa_f, Pb_f, V_max,2,5);
          Energy_col(i,1) = total_energy;
          minsep(sl,1)=3-C_ineq(2);
          dc=dc+1;
          else 
               x_sc=X_sc_trained(inp);
         x_sc2=X_sc_trained(inp2);
   
         
            X1=x_sc;
            X2=x_sc2;
            time_rec(sl,1)=toc;
            method(sl,1)=0;
             [total_energy]= original_motion(points(i),15,x_sc(1),x_sc2(1));
             Energy_no_col(i,1) = total_energy;
           [total_energy ,C_eq, C_ineq]= visual_sim_2_agent_sc_test(X1,X2,V1, V2, Pa_i, Pb_i, Pa_f, Pb_f, V_max,2,5);
          Energy_col(i,1) = total_energy;
           minsep(sl,1)=3-C_ineq(2);
           sc=sc+1;
          end 

      if minsep(sl,1)<1.5
          col=col+1
          i
      end


    close all
    end
end

% col=0;
% for i=1:length(minsep)
% if minsep(i,1)<1.5
% col=col+1;
% end
% end
% min(minsep)
   
lk=0;
for i=1:length(minsep)
    if minsep(i,1)<1.5
        lk=lk+1;
        nw(lk)=points(i);
        if method(i,1)==1;
            mt(lk,1)=0;
        else
            mt(lk,1)=1;
        end
    end
end

