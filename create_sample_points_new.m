function Points= create_sample_points_new(V_max,t_det,d_thresh,n_max,map_x)
%% function to creaat n_max samples that predicts a collision at tine t_det
d_min=d_thresh;
i=1;
%%
% Pa=(2*map_x)*rand(2,1) -map_x; % this point is constant for all the samples
Pa=[0;0];
% intialise the initial point and the velocity for agent A
% lhsdesign variables - theta_A, speed_A, delta_theta, speed_b
%%
% upper and lower bounds for the variables
Va_lb=5;
Va_ub=V_max;
Vb_lb=1;
Vb_ub=V_max;
theta_a_lb=0;
theta_a_ub=2*pi;
delta_theta_lb=0.01*pi;
delta_theta_ub=2*pi;
%%
% defining the upper and lower bounds
ub=[theta_a_ub Va_ub delta_theta_ub Vb_ub];
lb=[theta_a_lb Va_lb delta_theta_lb Vb_lb];
vec=lhsdesign(n_max,4);
%%
% vec(i,1) - heading angle for agent A
% vec(i,2) - speed of agent  A
% vec(i,3) - angle of aproach
% vec(i,4) - speed of agent B
%%
while i<=n_max
    vec(i,:)=vec(i,:).*(ub-lb)+lb;
    Points(i).Pa=Pa; % initial location for agent A
    Points(i).Va=[cos(vec(i,1));sin(vec(i,1))].*vec(i,2);
    
    theta_A=vec(i,1);
    theta_B=theta_A+vec(i,3);% heading angle for agent B
    
    Col_Pa= Points(i).Pa + Points(i).Va.*t_det;% collision point for agent A
    Col_Pb= d_min.*[cos(theta_B);sin(theta_B)] + Col_Pa; %collision point for agent B
    speed_B=vec(i,4);
    Points(i).Vb=speed_B.*[cos(theta_B); sin(theta_B)];
    Points(i).Pb=Col_Pb - Points(i).Vb.*t_det;
    
    i=i+1;
end
    
    