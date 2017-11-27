function op=main(sample_id)
% function op=main()
% sample_id=1;


%% specify the assumptions
V_max=15; % assumed maximum speed for a UAV
t_det=5; % collision detection time
UAV_size=.5; % radius of UAV
safety_factor=3;
d_thresh=UAV_size*safety_factor; % collision defining distance
n_max=1000; % number of samples


%% creating samples
% points=create_sample_points_new(V_max,t_det,d_thresh,n_max,map_x);
load('sample_new3.mat') % loads the sample points from the DOE
sl=0;
scnt=0;
dcnt=0;
% load('rchk.mat')
% for i=1:length(points)
i=sample_id;
%     close all
filename1=sprintf('../myfile_new_%d.mat',i);
filename2=sprintf('myfile_new_%d.mat',i);
    Pa_i=points(i).Pa;
    Pb_i=points(i).Pb;
    V1=points(i).Va;
    V2=points(i).Vb;
    Pa_f=Pa_i + V1.*t_det;
    Pb_f=Pb_i + V2.*t_det;
    [detect tcol]=col_det2(Pa_i,V1,Pb_i,V2);
    t_15(i,1)=time_to_min_dist_15(Pa_i,V1,Pb_i,V2);
    Pb_i=Pb_i-V2.*(t_det-t_15(i,1));
    if detect==1 %&& t_15(i,1)>0
%         tc1=transition_begin_time(V1,V2,Pa_i,Pb_i);
%         tc=tc1;
%         minsep_initial=original_path( V1, V2, Pa_i, Pb_i, Pa_f, Pb_f,15);
        [X1 fx1]=SC(V1,V2,Pa_i,Pb_i,Pa_f,Pb_f,V_max,t_det); % finds the optimal values for speed change
        [X2 fx2]=DC(V1,V2,Pa_i,Pb_i,Pa_f,Pb_f,V_max,t_det); % finds the optimal values for direction change
%         [X2 fx2]=SC(tc1,V1,V2,Pa_i,Pb_i,Pa_f,Pb_f,t_trans,tc,V_max,tcol);
% %         visual_sim(X,V1, V2, tc, Pa_i, Pb_i, Pa_f, Pb_f, V_max,tc1,t_trans,2,tcol);
%         sl=sl+1;
        data.Pa=Pa_i;
        data.Pb=Pb_i;
        data.Va=V1;
        data.Vb=V2;
        data.X1=X1;
        data.fx1=fx1;
        data.X2=X2;
        data.fx2=fx2;
%         if fx1>=50000
%             scnt=scnt+1;
%             data_scnt(scnt).Pa=Pa_i;
%         data_scnt.Pb=Pb_i;
%         data_scnt.Va=V1;
%         data_scnt.Vb=V2;
%         end
%         if fx2>=50000
%             dcnt=dcnt+1;
%             data_dcnt(dcnt).Pa=Pa_i;
%         data_dcnt(dcnt).Pb=Pb_i;
%         data_dcnt(dcnt).Va=V1;
%         data_dcnt(dcnt).Vb=V2;
%         end
%         
    end
% end

% my_unique_file_name = sprintf('my_file_name_%d.mat', labindex);
save(filename1,'data','points')
save(filename2,'data','points')
op=1;
end
%   fclose(main);
