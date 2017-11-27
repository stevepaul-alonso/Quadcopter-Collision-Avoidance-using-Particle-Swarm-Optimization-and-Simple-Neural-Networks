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
 load('not_working6.mat')
 points=nw;
for i=1:length(points)
     Pa_i=points(i).Pa;
    Pb_i=points(i).Pb;
    V1=points(i).Va;
    V2=points(i).Vb;
    Pa_f=Pa_i + V1.*t_det;
    Pb_f=Pb_i + V2.*t_det;
    [detect tcol]=col_det2(Pa_i,V1,Pb_i,V2);
    t_15(i,1)=time_to_min_dist_15(Pa_i,V1,Pb_i,V2);
    Pb_i=Pb_i-V2.*(t_det-t_15(i,1));
    speed(i,1)=norm(nw(i).Va);
    speed(i,2)=norm(nw(i).Vb);
%     load('separation.mat')
    if detect==1
        tic
        sl=sl+1;
        rel_speed=(V2)'-(V1)';
        rel_dist=(Pb_i)';
        theta=atan2(V2(2,1),V2(1,1))-atan2(V1(2,1),V1(1,1));
        theta2=atan2(V1(2,1),V1(1,1))-atan2(V2(2,1),V2(1,1));
        inp=[rel_speed rel_dist];
        inp2=[-rel_speed -rel_dist];
         inp_sc_A=[Pb_i' theta norm(V1) norm(V2)];
        inp_sc_B=[-Pb_i' theta norm(V2) norm(V1)];
%         x1=x1_sc_t2(inp);
%         x12=x1_dc_t2(inp);
%         x2=x2_sc_t2(inp);
%           x22=x2_dc_t2(inp);
         x_sc=X_sc_trained(inp_sc_A);
         x_sc2=X_sc_trained(inp_sc_B);
%          x_dc=X_dc_trained(inp);
%          x12=x_dc(1,1); x22=x_dc(1,2);
%          x12b=x1_dc_trained(inp2); x22b=x2_dc_trained(inp2);
        x12=x1_dc_trained(inp); x22=x2_dc_trained(inp);
         x12b=x1_dc_trained(inp2); x22b=x2_dc_trained(inp2);

         x1=x_sc(1,1); x2=x_sc(1,2);
          inp_dc=[inp x12 x22];
          inp_sc=[inp x1 x2];
          l1=inp_dc; l2=inp_sc;
%           min_dc=sep_dc_trained(inp_dc);
%           min_sc=sep_sc_trained(inp_sc);
          dist=norm(Pb_i);
          v1=norm(V1);
          v2=norm(V2);
         approach_angle(sl,1)=acosd(points(i).Vb'*points(i).Va/(norm(points(i).Va)*norm(points(i).Vb)));

%           approach_angle(sl,1)=rem((atan2(points(i).Vb(2,1),points(i).Vb(1,1))-atan2(points(i).Va(2,1),points(i).Va(1,1)))*180/pi + 360,180);
        
         
          in=[approach_angle(sl,1) v1 v2];
%              meth=predict(meth_pred,approach_angle);
%             meth=predict(meth_pred,in);
          meth=mt(i,1);
         meth_pred(i,1)=method_prediction(in);
          if meth_pred(i,1) >.5
           X1=[x12,x22];
           X2=[x12b,x22b];
           method(sl,1)=1;
           [total_energy ,C_eq, C_ineq]= visual_sim_2_agent(X1,X2,V1, V2, Pa_i, Pb_i, Pa_f, Pb_f, V_max,2,5);
          minsep(sl,1)=3-C_ineq(2);
          dc=dc+1;
          else 
            X1=[x1,x2];
            X2=x_sc2;
            method(sl,1)=0;
           [total_energy ,C_eq, C_ineq]= visual_sim_2_agent_sc(X1,X2,V1, V2, Pa_i, Pb_i, Pa_f, Pb_f, V_max,2,5);
           minsep(sl,1)=3-C_ineq(2);
           sc=sc+1;
          end 

                      


    close all
    end
end