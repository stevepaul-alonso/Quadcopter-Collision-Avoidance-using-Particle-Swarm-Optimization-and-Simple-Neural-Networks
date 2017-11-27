function sdot = quadEOM(t, s, controlhandle, trajhandle, params)




% convert state to quad stuct for control
current_state = stateToQd(s);

% Get desired_state

desired_state = trajhandle(t, current_state);

% get control outputs
[F, M] = controlhandle(t, current_state, desired_state, params);

% compute derivative
% sdot = quadEOM_readonly(t, s, F, M, params);


%%

A = [0.25,                      0, -0.5/params.arm_length;
     0.25,  0.5/params.arm_length,                      0;
     0.25,                      0,  0.5/params.arm_length;
     0.25, -0.5/params.arm_length,                      0];

prop_thrusts = A*[F;M(1:2)]; % Not using moment about Z-axis for limits
prop_thrusts_clamped = max(min(prop_thrusts, params.maxF/4), params.minF/4);
power=Power(prop_thrusts_clamped);
B = [                 1,                 1,                 1,                  1;
                      0, params.arm_length,                 0, -params.arm_length;
     -params.arm_length,                 0, params.arm_length,                 0];
F = B(1,:)*prop_thrusts_clamped;
M = [B(2:3,:)*prop_thrusts_clamped; M(3)];

% Assign states
x = s(1);
y = s(2);
z = s(3);
xdot = s(4);
ydot = s(5);
zdot = s(6);
qW = s(7);
qX = s(8);
qY = s(9);
qZ = s(10);
p = s(11);
q = s(12);
r = s(13);

quat = [qW; qX; qY; qZ];
%%
% q=quat;
% quat = quat./sqrt(sum(quat.^2));

% qahat(1,2) = -qt(4);
% qahat(1,3) = q(3);
% qahat(2,3) = -q(2);
% qahat(2,1) = q(4);
% qahat(3,1) = -q(3);
% qahat(3,2) = q(2);

% bRw = eye(3) + 2*qahat*qahat + 2*q(1)*qahat;
%%
bRw = QuatToRot(quat);


wRb = bRw';

% Acceleration
% accel = 1 / params.mass * (wRb * [0; 0; F] - [.25*xdot^2; .25*ydot^2; params.mass * params.gravity]);
accel = 1 / params.mass * (wRb * [0; 0; F] - [0; 0; params.mass * params.gravity]);

% Angular velocity
K_quat = 2; %this enforces the magnitude 1 constraint for the quaternion
quaterror = 1 - (qW^2 + qX^2 + qY^2 + qZ^2);
qdot = -1/2*[0, -p, -q, -r;...
             p,  0, -r,  q;...
             q,  r,  0, -p;...
             r, -q,  p,  0] * quat + K_quat*quaterror * quat;

% Angular acceleration
omega = [p;q;r];
pqrdot   = params.invI * (M - cross(omega, params.I*omega));
% for i=1:3
%     if norm(accel(i))>10
%         accel(i)=10*accel(i)/norm(accel(i));
%     end
% end


% Assemble sdot
sdot = zeros(13,1);
sdot(1)  = xdot;
sdot(2)  = ydot;
sdot(3)  = zdot;
sdot(4)  = accel(1);
sdot(5)  = accel(2);
sdot(6)  = accel(3);
sdot(7)  = qdot(1);
sdot(8)  = qdot(2);
sdot(9)  = qdot(3);
sdot(10) = qdot(4);
sdot(11) = pqrdot(1);
sdot(12) = pqrdot(2);
sdot(13) = pqrdot(3);
sdot(14:17)=power;


end
