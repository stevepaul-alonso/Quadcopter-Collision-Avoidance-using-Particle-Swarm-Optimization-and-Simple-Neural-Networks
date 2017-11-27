function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%


persistent err1 err2
if t==0
    err1=zeros(3,1);
    err2=zeros(3,2);
end


kp=3; kd=.1; ki=0;
Kp_phi=kp;
Kp_theta=kp;
Kp_psi=kp;
Kd_phi=kd;
Kd_theta=kd;
Kd_psi=kd;
Ki_phi=ki;
Ki_theta=ki;
Ki_psi=ki;
K_att=[Kp_phi Kd_phi;Kp_theta Kd_theta;Kp_psi Kd_psi];


Kp1=10; Kp2=10; Kp3=15; 
Kd1=20; Kd2=20; Kd3=20;
Ki1=0; Ki2=0; Ki3=0; 
% % chng=[state.rot
Kpz=20; Kvz=20; Kiz=20;
err1=des_state.pos-state.pos; 
r1_des_dotdot= des_state.acc(1,1) + Kd1*(des_state.vel(1,1) - state.vel(1,1)) + Kp1*(des_state.pos(1,1) - state.pos(1,1)) + Ki1*err1(1,1);
r2_des_dotdot= des_state.acc(2,1) + Kd2*(des_state.vel(2,1) - state.vel(2,1)) + Kp2*(des_state.pos(2,1) - state.pos(2,1)) + Ki2*err1(2,1);
r3_des_dotdot= des_state.acc(3,1) + Kd3*(des_state.vel(3,1) - state.vel(3,1)) + Kp3*(des_state.pos(3,1) - state.pos(3,1))+ Ki3*err1(3,1); 

% Thrust
F = 0;
F=params.mass.*(params.gravity + r3_des_dotdot); %Kvz*(-state.vel(3)) - Kpz*(state.pos(3)-des_state.pos(3)));
% Moment
M = zeros(3,1);
phi_des= ((r1_des_dotdot*sin(des_state.yaw)) - r2_des_dotdot*cos(des_state.yaw))/params.gravity;
theta_des= ((r1_des_dotdot*cos(des_state.yaw)) + r2_des_dotdot*sin(des_state.yaw))/params.gravity;
p_des=0;
q_des=0;
psi_des=des_state.yaw;
r_des=des_state.yawdot;
err2=[phi_des;theta_des;psi_des]-state.rot;
u21= Kp_phi*(phi_des - state.rot(1)) + Kd_phi*(p_des-state.omega(1))+ Ki_phi*err2(1,1);
u22= Kp_theta*(theta_des - state.rot(2)) + Kd_theta*(q_des-state.omega(2))+ Ki_theta*err2(2,1);
u23= Kp_psi*(psi_des - state.rot(3)) + Kd_psi*(r_des-state.omega(3))+ Ki_psi*err2(3,1);
M=[u21;u22;u23];




end
