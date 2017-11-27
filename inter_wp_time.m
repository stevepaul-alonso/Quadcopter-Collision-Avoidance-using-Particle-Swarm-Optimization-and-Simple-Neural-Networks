function [waypoints time1 waypoints2 time2]=inter_wp_time(Xn,varargin)
% function[total_energy ,C_eq, C_ineq]= running_sim()

% load('nww.mat')
% Xn=[0.8189,1.5158];

V1 = varargin{1,1};
V2 = varargin{1,2};
tc = varargin{1,3};
Pa_i = varargin{1,4};
Pb_i = varargin{1,5};
Pa_f = varargin{1,6};
Pb_f = varargin{1,7};
V_max = varargin{1,8};
tc1=varargin{1,9};
t_trans=varargin{1,10};
method=varargin{1,11};
t_col=varargin{1,12};

mu=Xn(1);

t_ac=mu*tc1; %time action starts
P_ac= Pa_i + V1.*t_ac; %point at which agent a changes speed
P_bc= Pb_i + V2.*t_ac; %point at which agent b changes speed
% t_cac= tc1+ (1-mu)*tc1; %time action completes
% t_cac= 10-t_ac;
t_cac=t_col+ (t_col-t_ac);

% x1=Pa_i(1,1); x2=Pa_f(1,1); y1=Pa_i(2,1); y2=Pa_f(2,1);
% x3=Pb_i(1,1); x4=Pb_f(1,1); y3=Pb_i(2,1); y4=Pb_f(2,1);
% 
% den=det([det([x1 1;x2 1]) det([y1 1;y2 1]); det([x3 1;x4 1]) det([y3 1;y4 1])]);
% Px=det([det([x1 y1;x2 y2]) det([x1 1; x2 1]); det([x3 y3; x4 y4]) det([x3 1; x4 1])])/den;
% Py=det([det([x1 y1;x2 y2]) det([y1 1; y2 1]); det([x3 y3; x4 y4]) det([y3 1; y4 1])])/den;
% t_cross_a=norm([Px;Py]-Pa_i)/norm(V1);
% t_cross_b=norm([Px;Py]-Pb_i)/norm(V2);

% tm1=(t_ac+t_cross_a)/2;
% tm2=(t_ac+t_cross_b)/2;
tm1=(t_ac+t_cac)/2;
tm2=(t_ac+t_cac)/2;

if method==1 % for speed change
  v=Xn(2);
  va=v; vb=v;
  if norm(V1)>norm(V2)
      va= -v;
      vb=v;
  else
      va= v;
      vb=-v;
  end

  V_ac= (norm(V1)+va).*V1./norm(V1);
  V_bc= (norm(V2)+vb).*V2./norm(V2);

  P_cac= P_ac + V_ac.*(t_cac-t_ac);
  P_cbc= P_bc + V_bc.*(t_cac-t_ac);
  tm=(t_cac+t_ac)/2;


  P_am=P_ac + V_ac.*(tm1-t_ac);
  P_bm=P_bc + V_bc.*(tm2-t_ac);

  waypoints=[[Pa_i]' 0; [P_ac]' 0; [P_am]' 0; [P_cac]' 0; [Pa_f]' 0]';
  waypoints2=[[Pb_i]' 0; [P_bc]' 0; [P_bm]' 0; [P_cbc]' 0; [Pb_f]' 0]';
   time1=[0;t_ac;tm1;t_cac;t_trans];
  time2=[0;t_ac;tm2;t_cac;t_trans];
end

if method==2 % for direction change
  theta=Xn(2);
  new_speed_a = norm(V1)/cos(theta);
  new_speed_b = norm(V2)/cos(theta);
  head_init_a = atan2(V1(2,1), V1(1,1));
  head_init_b = atan2(V2(2,1), V2(1,1));
  head_new_a = head_init_a + theta;
  head_new_b = head_init_b + theta;
  Va_new1 = [new_speed_a*cos(head_new_a);new_speed_a*sin(head_new_a)];
  Vb_new1 = [new_speed_b*cos(head_new_b);new_speed_b*sin(head_new_b)];

  Pa_inter= P_ac + (tm1 - t_ac).*Va_new1;
  Pb_inter= P_bc + (tm2 - t_ac).*Vb_new1;
  V_ac=Va_new1;
  V_bc=Vb_new1;



%   P_cac= P_ac + V1.*(t_cross_a-t_ac+1);
%   P_cbc= P_bc + V2.*(t_cross_b-t_ac+1);
  P_cac= P_ac + V1.*(t_cac-t_ac);
  P_cbc= P_bc + V2.*(t_cac-t_ac);

  V_cac = (P_cac- Pa_inter)./(t_cac-tm1);
  V_cbc = (P_cbc- Pb_inter)./(t_cac-tm2);
  if t_cac>=t_trans
      if tm1>=t_trans
          tm1=(t_trans+t_ac)/2;
          t_cac=(t_trans+tm1)/2;
           Pa_inter= P_ac + (tm1 - t_ac).*Va_new1;
           Pb_inter= P_bc + (tm1 - t_ac).*Vb_new1;
           V_cac = (Pa_f- Pa_inter)./(t_trans-tm1);
           V_cbc = (Pb_f- Pb_inter)./(t_trans-tm1);
          P_cac= Pa_inter + V_cac.*(t_cac-tm1);
          P_cbc= Pb_inter + V_cbc.*(t_cac-tm1);  
           waypoints=[[Pa_i]' 0; [P_ac]' 0; [Pa_inter]' 0; [P_cac]' 0; [Pa_f]' 0]';
         waypoints2=[[Pb_i]' 0; [P_bc]' 0; [Pb_inter]' 0; [P_cbc]' 0; [Pb_f]' 0]';
         time1=[0;t_ac;tm1;t_cac;t_trans];
        time2=[0;t_ac;tm1;t_cac;t_trans];%
      else
        V_cac = (Pa_f- Pa_inter)./(t_trans-tm1);
        V_cbc = (Pb_f- Pb_inter)./(t_trans-tm1);
      t_cac=(t_trans+tm1)/2;
      P_cac= Pa_inter + V_cac.*(t_cac-tm1);
      P_cbc= Pb_inter + V_cbc.*(t_cac-tm1);
     waypoints=[[Pa_i]' 0; [P_ac]' 0; [Pa_inter]' 0; [P_cac]' 0; [Pa_f]' 0]';
     waypoints2=[[Pb_i]' 0; [P_bc]' 0; [Pb_inter]' 0; [P_cbc]' 0; [Pb_f]' 0]';
     time1=[0;t_ac;tm1;t_cac;t_trans];
     time2=[0;t_ac;tm1;t_cac;t_trans];%
     end
  else
  waypoints=[[Pa_i]' 0; [P_ac]' 0; [Pa_inter]' 0; [P_cac]' 0; [Pa_f]' 0]';
  waypoints2=[[Pb_i]' 0; [P_bc]' 0; [Pb_inter]' 0; [P_cbc]' 0; [Pb_f]' 0]';
  time1=[0;t_ac;tm1;t_cac;t_trans];
  time2=[0;t_ac;tm1;t_cac;t_trans];
  end
end
